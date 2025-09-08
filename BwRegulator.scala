package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
//import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}

import freechips.rocketchip.tile.{BRUTileIO, BRUTileAccessIO}

// case class BRUParams (
//   address: BigInt,
//   nDomains: Int,
//   withMonitor: Boolean  // avoid including this when using multiple mempress, too many edges and the regmap gets too large
// )

//case object BRUKey extends Field[Option[BRUParams]](None)

class BwRegulator() (implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bru",Seq("bru"))

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(0x20000000, 0x7ff)),
    device = device,
    beatBytes = 8)

  val adapterNode = TLAdapterNode()
  val ioNode = Seq.fill(4)(BundleBridgeSource(() => new BRUTileIO(p(SubsystemBankedCoherenceKey).nBanks)))
  val coreAccessNode = Seq.fill(4)(BundleBridgeSink[BRUTileAccessIO](Some(() => Flipped(new BRUTileAccessIO(p(SubsystemBankedCoherenceKey).nBanks)))))
  lazy val module = new BwRegulatorModule(this)
}

class BwRegulatorModule(outer: BwRegulator) extends LazyModuleImp(outer)
{
  // A TLAdapterNode has equal number of input and output edges
  val n = outer.adapterNode.in.length
  println(s"Number of edges into BRU: $n")

  val nDomains = 4
  require(nDomains <= 32) //Limit for regmapper addresses

  val withMonitor = false

  val nBanks = p(SubsystemBankedCoherenceKey).nBanks
  val numBankBits = log2Ceil(nBanks)

  val throttleIO = outer.ioNode.map(_.bundle)
  val accessIO = outer.coreAccessNode.map(_.bundle)

  val memBase = p(ExtMem).get.master.base.U
  val wPeriod = 25 // for max 33.5ms period, F = 1GHz
  val w = wPeriod - 3 // it can count up to a transaction per 8 cycles when window size is set to max
  var clientNames = new Array[String](n)

  val enBRUGlobal = RegInit(false.B)
  val countInstFetch = RegInit(true.B)
  //val enWbThrottle = RegInit(false.B)
  val periodCntr = Reg(UInt(wPeriod.W))
  val periodLen = Reg(UInt(wPeriod.W))
  val bankReadCntrs = Seq.fill(nDomains)(RegInit(VecInit(Seq.fill(nBanks)(0.U(w.W)))))
  val maxReads = Reg(Vec(nDomains, UInt(w.W)))
  //val wbCntrs = Reg(Vec(nDomains, UInt(w.W)))
  //val maxWbs = Reg(Vec(nDomains, UInt(w.W)))
  val clientRegEnable = Reg(Vec(n, Bool()))
  val clientDomainIds = Reg(Vec(n, UInt(log2Ceil(nDomains).W)))
  val coreAcquireActive = Wire(Vec(n, Bool()))
  //val coreWbActive = Wire(Vec(n, Bool()))
  val doesAccessBank = Seq.fill(n)(Wire(Vec(nBanks, Bool())))
  val throttleReadDomainBanks = VecInit(Seq.fill(nDomains)(VecInit(Seq.fill(nBanks)(WireInit(Bool(), false.B)))))

  val perfEnable = RegInit(false.B)
  // It is not required to reset these counters but we keep it for now as it helps to close timing
  //  more easily in PnR
  val aCounters = if ( withMonitor ) Some(Seq.fill(n)(RegInit(VecInit(Seq.fill(nBanks)(0.U(64.W)))))) else None
  val cCounters = if ( withMonitor ) Some(Seq.fill(n)(RegInit(VecInit(Seq.fill(nBanks)(0.U(64.W)))))) else None

  val periodCntrReset = periodCntr >= periodLen
  periodCntr := Mux(periodCntrReset || !enBRUGlobal, 0.U, periodCntr + 1.U)

  // generator loop for domains
  for (i <- 0 until nDomains) {

    for (j <- 0 until nBanks) {
      // bit vectors for clients that are enabled & access mem in the current cycle & are assigned to domain i & are in accssessing bank j
      val clientAcquireActBankMasked = (clientDomainIds zip (coreAcquireActive zip doesAccessBank)).map { case (d, (act, bank)) => d === i.U && act && bank(j) }

      // should be able to reduce or the masks as sytem bus only allows one request per cycle
      bankReadCntrs(i)(j) := Mux(enBRUGlobal, clientAcquireActBankMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, bankReadCntrs(i)(j)), 0.U)

      throttleReadDomainBanks(i)(j) := bankReadCntrs(i)(j) >= maxReads(i)
    }

    // TODO: get wb throttling to work in Boom
    // leaving this here to legacy, doesn't currently do anything
    // val coreWbActMasked = (clientDomainIds zip coreWbActive).map { case (d, act) => d === i.U && act }
    // wbCntrs(i) := Mux(enBRUGlobal, coreWbActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, wbCntrs(i)), 0.U)
    // throttleDomainWb(i) := wbCntrs(i) >= maxWbs(i)
  }

  //generator loop for client edges
  for (i <- 0 until n) {
    val (out, edge_out) = outer.adapterNode.out(i)
    val (in, edge_in) = outer.adapterNode.in(i)

    //val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
    //val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
    val aIsAcquire = accessIO(i).didFire
    // ReleaseData or ProbeAckData cause a PutFull in Broadcast Hub
    val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

    val aIsRead = aIsAcquire //|| (aIsInstFetch && countInstFetch)

    coreAcquireActive(i) := clientRegEnable(i) && accessIO(i).didFire //out.a.fire && aIsRead

    //per bank support
    //do we access bank j
    val bankBits = Wire(UInt(nBanks.W))
    //bankBits := in.a.bits.address(6+numBankBits-1, 6) // Can we make 6 (cache line boundary) not a magic number?
    bankBits := accessIO(i).bank
    for (j <- 0 until nBanks) {
      doesAccessBank(i)(j) := bankBits === j.U

      aCounters match {
        case None => // nothing
        case Some(aCounts) => aCounts(i)(j) := Mux(perfEnable, 
                        ((out.a.fire) && (aIsRead) && doesAccessBank(i)(j)) + aCounts(i)(j), 0.U)
      }
      
      cCounters match {
        case None => // nothing
        case Some(cCounts) => cCounts(i)(j) := Mux(perfEnable,
                        ((edge_out.done(out.c) && cIsWb) && doesAccessBank(i)(j)) + cCounts(i)(j), 0.U)
      }

    }

    out <> in

    for ( j <- 0 until nBanks ) {
        throttleIO(i).nThrottle(j) := throttleReadDomainBanks(clientDomainIds(i))(j) && clientRegEnable(i) && enBRUGlobal
        //throttleIO(i).nThrottle(j) := false.B
    }

    when (enBRUGlobal && clientRegEnable(i)) {
      for (j <- 0 until nBanks ) {
        when ( ( throttleReadDomainBanks(clientDomainIds(i))(j) && doesAccessBank(i)(j) ) && aIsRead ) {
        //   out.a.valid := false.B
        //   in.a.ready := false.B
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

  val enBRUGlobalRegField = Seq(0 -> Seq(
    RegField(enBRUGlobal.getWidth, enBRUGlobal,
      RegFieldDesc("enBRUGlobal", "Enable BRU Global"))))

    val settingsRegField = Seq(8 -> Seq (
        RegField(countInstFetch.getWidth, countInstFetch,
            RegFieldDesc("countInstrFetch", "Toggle fetch counting"))))

    val periodLenRegField = Seq(16 -> Seq (
        RegField(periodLen.getWidth, periodLen,
            RegFieldDesc("periodLen", "Set regulation period"))))

    val maxReadsRegField = maxReads.zipWithIndex.map { case (register, i) =>
        (24 + i * 8) -> Seq(RegField(register.getWidth, register,
            RegFieldDesc(s"maxRead$i", s"Max reads for domain $i")))}

    val clientRegEnableRegField = Seq((24 + nDomains * 8) -> clientRegEnable.zipWithIndex.map { case (client, i) =>
        RegField(client.getWidth, client, RegFieldDesc(s"client${i}En", s"Reg enable for client$i")) })

    val domainIdField = clientDomainIds.zipWithIndex.map { case(domain, i) =>
        (48 + nDomains * 8 + i * 8) -> Seq(RegField(domain.getWidth, domain, RegFieldDesc(s"domainId$i", s"Client $i domain ID"))) }

  val perfEnField = Seq((48 + 3 * 8 * nDomains + n * 8) -> Seq(
    RegField(perfEnable.getWidth, perfEnable,
      RegFieldDesc("perfEnable", "perfEnable"))))

  val mmreg = enBRUGlobalRegField ++ settingsRegField ++ periodLenRegField ++ maxReadsRegField ++ clientRegEnableRegField ++
      domainIdField ++ perfEnField

  (aCounters, cCounters) match {
    case (Some(aCounts), Some(cCounts)) => {
      val bankReadCountersField = aCounts.zipWithIndex.flatMap { case (banks, i) =>
        banks.zipWithIndex.map { case (bank, j) =>
          val addr = 8 * (8 + 3 * nDomains + n + i * nBanks + j)
          addr -> Seq(
            RegField(bank.getWidth, bank,
              RegFieldDesc(s"bankCountR${i * nBanks + j}", s"Bank Read counter"))
          )
        }
      }

      val bankWriteCountersField = cCounts.zipWithIndex.flatMap { case (banks, i) =>
        banks.zipWithIndex.map { case (bank, j) =>
          val addr = 8 * (8 + 3 * nDomains + n + n * nBanks + i * nBanks + j)
          addr -> Seq(
            RegField(bank.getWidth, bank,
              RegFieldDesc(s"bankCountW${i * nBanks + j}", s"Bank Write counter"))
          )
        }
      }

      outer.regnode.regmap(mmreg ++ bankReadCountersField ++ bankWriteCountersField: _*)
    }
    case _ => outer.regnode.regmap(mmreg: _*)
  }

  println("Bandwidth regulation (BRU):")
  for (i <- clientNames.indices)
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
            bwreg.regnode := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ 
        }
    }
}

// trait CanHavePeripheryBRU { this: BaseSubsystem =>
//   private val portName = "llc-bru"

//   val BwRegulator = p(BRUKey) match {
//     case Some(params) => {
//       val BwRegulator = LazyModule(new BwRegulator(params, "llc")(p))

//       pbus.coupleTo(portName) { 
//         BwRegulator.regnode := 
//         TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }

//       Some(BwRegulator)
//     }
//     case None => None
//   }
// }

// class WithBRU(address: BigInt = 0x20000000L, nDomains: Int = 4, withMonitor: Boolean = false) extends Config((_, _, _) => {
//   case BRUKey => Some(BRUParams(address = address, nDomains = nDomains, withMonitor = withMonitor))
// })
