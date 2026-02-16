package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
//import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}

import freechips.rocketchip.tile.{BRUTileIO, BRUTileAccessIO}

case object BRUKey extends Field[Option[BwControllerParams]](None)

class BwRegulator(params: BwControllerParams) (implicit p: Parameters) extends LazyModule
{
  var resourcesOpt: Option[ResourceBindings] = None
  val regulationDevice = new SimpleDevice("llc-memory-reg",Seq("rsicv,cbqri-bandwidth-cache", "rsicv,cbqri-bandwidth")) {
    def ofInt(x: Int) = Seq(ResourceInt(BigInt(x)))

    override def describe(resources: ResourceBindings): Description = {
      resourcesOpt = Some(resources)

      val Description(name, mapping) = super.describe(resources)
      val extra = Map(
        "riscv,cbqri-rcid" -> ofInt(params.nRCID),
        "riscv,cbqri-mcid" -> ofInt(params.nMCID))
      Description(name, mapping ++ extra)
    }
  }

  val adapterNode = TLAdapterNode()
  // NOTE: we currently assume that nDomains == number of cores
  // if you decide to stray from this, change params.nDomains below to be equal to number of cores
  val ioNode = Seq.fill(1)(BundleBridgeSource(() => new BRUTileIO(p(SubsystemBankedCoherenceKey).nBanks)))
  // val coreAccessNode = Seq.fill(4)(BundleBridgeSink[BRUTileAccessIO](Some(() => Flipped(new BRUTileAccessIO(p(SubsystemBankedCoherenceKey).nBanks)))))

  val mmio = LazyModule(new CBQRIBwController(regulationDevice, params))
  lazy val module = new BwRegulatorModule(this, params)
}

class BwRegulatorModule(outer: BwRegulator, params: BwControllerParams) extends LazyModuleImp(outer)
{
  // A TLAdapterNode has equal number of input and output edges
  val n = outer.adapterNode.in.length
  println(s"Number of edges into BRU: $n")

  val nRCID = params.nRCID
  val nMCID = params.nMCID
  val nbwblks = params.nbwblks
  val mrbwb = params.mrbwb

  val nBanks = p(SubsystemBankedCoherenceKey).nBanks
  val numBankBits = log2Ceil(nBanks)

  val throttleIO = outer.ioNode.map(_.bundle)
  // val accessIO = outer.coreAccessNode.map(_.bundle)

  private val mmio = outer.mmio.module

  val memBase = p(ExtMem).get.master.base.U
  val wPeriod = 25 // for max 33.5ms period, F = 1GHz
  var clientNames = new Array[String](n)

  val countInstFetch = RegInit(true.B)
  val periodCntr = Reg(UInt(wPeriod.W))
  val bankReadCntrs = Seq.fill(nRCID)(RegInit(VecInit(Seq.fill(nBanks)(0.U(log2Ceil(nbwblks).W)))))
  val maxReads = Reg(Vec(nRCID, UInt(log2Ceil(nbwblks).W)))
  val coreAcquireActive = Wire(Vec(n, Bool()))
  val coreAcquireRCID = Wire(Vec(n, UInt(log2Ceil(nRCID).W)))
  val coreAcquireMCID = Wire(Vec(n, UInt(log2Ceil(nMCID).W)))
  val doesAccessBank = Seq.fill(n)(Wire(Vec(nBanks, Bool())))
  val throttleReadDomainBanks = RegInit(VecInit(Seq.fill(nRCID)(VecInit(Seq.fill(nBanks)(false.B)))))

  val mcidCounters = RegInit(VecInit(Seq.fill(nMCID)(0.U(62.W))))
  val mcidEvts = RegInit(VecInit(Seq.fill(nMCID)(BcMonCtlEvent.NONE)))
  val s_mcid_hold :: s_mcid_count :: s_mcid_reset :: Nil = Enum(3)
  val mcidStates = RegInit(VecInit(Seq.fill(nMCID)(s_mcid_hold)))

  mmio.io.bc_mon_resp.valid := false.B
  mmio.io.bc_mon_resp.bits.status := BcMonCtlStatus.OK
  mmio.io.bc_mon_resp.bits.hasData := false.B
  mmio.io.bc_mon_resp.bits.data := 0.U
  when ( mmio.io.bc_mon_command.valid ) {
    val opEnum = mmio.io.bc_mon_command.bits.op
    val opValid = ( opEnum === BcCtlOp.CONFIG ) || ( opEnum === BcCtlOp.READ )
    val evtEnum = mmio.io.bc_mon_command.bits.event
    val evtValid = ( evtEnum === BcMonCtlEvent.NONE ) || ( evtEnum === BcMonCtlEvent.READ_WRITE ) ||
                ( evtEnum === BcMonCtlEvent.READ_ONLY ) || ( evtEnum === BcMonCtlEvent.WRITE_ONLY )
    val mcid = mmio.io.bc_mon_command.bits.mcid
    val mcidValid = mcid < nMCID.U

    mmio.io.bc_mon_resp.valid := true.B
    when ( opValid && mcidValid && evtValid ) {
      switch ( opEnum ) {
        is ( BcCtlOp.CONFIG ) {
          when ( evtEnum =/= BcMonCtlEvent.NONE ) {
            mcidStates(mcid) := s_mcid_reset
          } .otherwise {
            mcidStates(mcid) := s_mcid_hold
          }
          mcidEvts(mcid) := evtEnum
          mmio.io.bc_mon_resp.bits.status := BcMonCtlStatus.OK
        }
        is ( BcCtlOp.READ ) {
          mmio.io.bc_mon_resp.bits.hasData := true.B
          mmio.io.bc_mon_resp.bits.data := mcidCounters(mcid)
          mmio.io.bc_mon_resp.bits.status := BcMonCtlStatus.OK
        }
      }
    } .elsewhen ( !mcidValid ) {
      mmio.io.bc_mon_resp.bits.status := BcMonCtlStatus.INVALID_MCID
    } .elsewhen ( !evtValid ) {
      mmio.io.bc_mon_resp.bits.status := BcMonCtlStatus.INVALID_EVT_ID
    }.otherwise {
      mmio.io.bc_mon_resp.bits.status := BcMonCtlStatus.INVALID_OP
    }
  }

  mmio.io.bc_alloc_ctl_resp.valid := false.B
  mmio.io.bc_alloc_ctl_resp.bits.status := BcAllocCtlStatus.OK
  mmio.io.bc_alloc_ctl_resp.bits.hasData := false.B
  mmio.io.bc_alloc_ctl_resp.bits.data := 0.U
  when ( mmio.io.bc_alloc_ctl_command.valid ) {
    val opEnum = mmio.io.bc_alloc_ctl_command.bits.op
    val opValid = ( opEnum === BcCtlOp.CONFIG ) || ( opEnum === BcCtlOp.READ )
    val rcid = mmio.io.bc_alloc_ctl_command.bits.rcid
    val rcidValid = rcid < nRCID.U
    
    val rbwb = mmio.io.bc_alloc_ctl_command.bits.rbwb

    mmio.io.bc_alloc_ctl_resp.valid := true.B
    when ( opValid && rcidValid ) {
      switch ( opEnum ) {
        is ( BcCtlOp.CONFIG ) {
          val rbwbValid = rbwb > 0.U && rbwb <= mrbwb.U
          when ( rbwbValid ) {
            maxReads(rcid) := rbwb
            mmio.io.bc_alloc_ctl_resp.bits.status := BcAllocCtlStatus.OK
          } .otherwise {
            mmio.io.bc_alloc_ctl_resp.bits.status := BcAllocCtlStatus.INVALID_BWB
          }
        }
        is ( BcCtlOp.READ ) {
          mmio.io.bc_alloc_ctl_resp.bits.hasData := true.B
          mmio.io.bc_alloc_ctl_resp.bits.data := maxReads(rcid)
          mmio.io.bc_alloc_ctl_resp.bits.status := BcAllocCtlStatus.OK
        }
      }
    } .elsewhen ( !rcidValid ) {
      mmio.io.bc_alloc_ctl_resp.bits.status := BcAllocCtlStatus.INVALID_RCID
    }.otherwise {
      mmio.io.bc_alloc_ctl_resp.bits.status := BcAllocCtlStatus.INVALID_OP
    }
  }

  val periodCntrReset = periodCntr >= mmio.io.periodLen
  periodCntr := Mux(periodCntrReset || !mmio.io.enGlobal, 0.U, periodCntr + 1.U)

  // generator loop for mcid
  for ( i <- 0 until nMCID ) {
    val clientAcquireActMask = (coreAcquireMCID zip coreAcquireActive).map { case (mcid, act) => mcid === i.U && act }
    val shoudIncAcquire = clientAcquireActMask.reduce(_||_)

    when ( mcidStates(i) === s_mcid_count ) {
      mcidCounters(i) := shoudIncAcquire + mcidCounters(i)
    } .elsewhen ( mcidStates(i) === s_mcid_hold ) {
      mcidCounters(i) := mcidCounters(i)
    } .elsewhen ( mcidStates(i) === s_mcid_reset ) {
      mcidCounters(i) := 0.U
      mcidStates(i) := s_mcid_count
    }
  }

  // generator loop for rcid
  for ( i <- 0 until nRCID ) {
    for ( j <- 0 until nBanks ) {
      // bit vectors for clients that are enabled & access mem in the current cycle & are assigned to domain i & are in accssessing bank j
      val clientAcquireActBankMasked = (coreAcquireRCID zip (coreAcquireActive zip doesAccessBank)).map { 
        case (rcid, (act, bank)) => rcid === i.U && act && bank(j) 
      }

      // should be able to reduce or the masks as sytem bus only allows one request per cycle
      val shouldIncAcquire = clientAcquireActBankMasked.reduce(_||_)
      val nextCntAcquire = Mux(mmio.io.enGlobal, Mux(!periodCntrReset, bankReadCntrs(i)(j) + shouldIncAcquire, shouldIncAcquire), 0.U)

      bankReadCntrs(i)(j) := nextCntAcquire
      throttleReadDomainBanks(i)(j) := Mux(!mmio.io.enGlobal || periodCntrReset, false.B, nextCntAcquire >= maxReads(i))
    }
  }

  //generator loop for client edges
  for ( i <- 0 until n ) {
    val (out, edge_out) = outer.adapterNode.out(i)
    val (in, edge_in) = outer.adapterNode.in(i)

    val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
    val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
    val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

    val aIsRead = aIsAcquire || ( aIsInstFetch && countInstFetch )

    coreAcquireActive(i) := in.a.fire && aIsRead
    coreAcquireRCID(i) := in.a.bits.rcid
    coreAcquireMCID(i) := in.a.bits.mcid

    //per bank support
    //do we access bank j
    val bankBits = Wire(UInt(nBanks.W))
    bankBits := in.a.bits.address(6+numBankBits-1, 6) // Can we make 6 (cache line boundary) not a magic number?
    for ( j <- 0 until nBanks ) {
      doesAccessBank(i)(j) := bankBits === j.U
    }

    when ( coreAcquireActive(i) ) {
      printf("RCID %d | MCID %d\n", coreAcquireRCID(i), coreAcquireMCID(i))
    }

    out <> in

    for ( j <- 0 until nBanks ) {
        //throttleIO(i).nThrottle(j) := throttleWriteDomainBanks(clientDomainIds(i))(j) && mmio.io.enGlobal
        throttleIO(i).nThrottle(j) := false.B
    }

    when ( mmio.io.enGlobal ) {
      for (j <- 0 until nBanks ) {
        when ( ( throttleReadDomainBanks(coreAcquireRCID(i))(j) && doesAccessBank(i)(j) ) && aIsRead ) {
           out.a.valid := false.B
           in.a.ready := false.B
        }
      }
    }

    // Hacky solution to handle different client types, better way to do this?
    if ( edge_in.client.clients.size > 2 ) {
      // core dcache and icache case
      clientNames(i) = edge_in.client.clients(0).name + ", " + edge_in.client.clients(2).name
    } else {
      // RoCC case
      clientNames(i) = edge_in.client.clients(0).name
    }
  }

  println("Bandwidth regulation (BRU):")
  for ( i <- clientNames.indices )
    println(s"  $i => ${clientNames(i)}")
}

trait CanHavePeripheryBRU {
    val BwRegulator: Option[BwRegulator]
}

trait CanHaveBRU { this: BaseSubsystem =>
    private val pbus = locateTLBusWrapper(PBUS)
    private val sbus = locateTLBusWrapper(SBUS)

    private val portName = "bru-mmio"

    sbus.BwRegulator.map { bwreg => 
        pbus.coupleTo(portName) {
            bwreg.mmio.regnode := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ 
        }
    }
}

class WithBRU(address: BigInt = 0x20000000L, nRCID: Int = 64, nMCID: Int = 64,
             ver: Int = 1, nbwblks: Int = 65535, rpfx: Boolean = false, p: Int = 0, mrbwb: Int = 52428)
extends Config((_, _, _) => {
  case BRUKey => {
    assert(nRCID <= 64) // interconnect limits for now
    assert(nMCID <= 64)
    Some(BwControllerParams(
      address = address, 
      nRCID = nRCID,
      nMCID = nMCID, 

      ver = ver,
      nbwblks = nbwblks,
      rpfx = rpfx,
      p = p,
      mrbwb = mrbwb
    ))
  }
})
