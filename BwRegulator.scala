package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
//import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}

import freechips.rocketchip.tile.{BRUTileIO, BRUTileAccessIO}

case class BRUParams (
  address: BigInt,
  nDomains: Int,
  withMonitor: Boolean
)

case object BRUKey extends Field[Option[BRUParams]](None)

class BwRegulator(params: BRUParams) (implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bru",Seq("bru"))

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(params.address, 0x7ff)),
    device = device,
    beatBytes = 8)

  val adapterNode = TLAdapterNode()
  val ioNode = Seq.fill(params.nDomains)(BundleBridgeSource(() => new BRUTileIO(p(SubsystemBankedCoherenceKey).nBanks)))
  // val coreAccessNode = Seq.fill(4)(BundleBridgeSink[BRUTileAccessIO](Some(() => Flipped(new BRUTileAccessIO(p(SubsystemBankedCoherenceKey).nBanks)))))
  lazy val module = new BwRegulatorModule(this, params)
}

class BwRegulatorModule(outer: BwRegulator, params: BRUParams) extends LazyModuleImp(outer)
{
  // A TLAdapterNode has equal number of input and output edges
  val n = outer.adapterNode.in.length
  println(s"Number of edges into BRU: $n")

  val nDomains = params.nDomains
  require(nDomains <= 32) //Limit for regmapper addresses

  val withMonitor = params.withMonitor

  val nBanks = p(SubsystemBankedCoherenceKey).nBanks
  val numBankBits = log2Ceil(nBanks)

  val throttleIO = outer.ioNode.map(_.bundle)
  // val accessIO = outer.coreAccessNode.map(_.bundle)

  val memBase = p(ExtMem).get.master.base.U
  val wPeriod = 25 // for max 33.5ms period, F = 1GHz
  val w = wPeriod - 3 // it can count up to a transaction per 8 cycles when window size is set to max
  var clientNames = new Array[String](n)

  val enBRUGlobal = RegInit(false.B)
  val countInstFetch = RegInit(true.B)
  val periodCntr = Reg(UInt(wPeriod.W))
  val periodLen = Reg(UInt(wPeriod.W))
  val readCntrs = Seq.fill(nDomains)(RegInit(0.U(w.W)))
  val maxReads = Reg(Vec(nDomains, UInt(w.W)))
  // val bankWriteCntrs = Seq.fill(nDomains)(RegInit(VecInit(Seq.fill(nBanks)(0.U(w.W)))))
  // val maxWrites = Reg(Vec(nDomains, UInt(w.W)))
  val clientRegEnable = Reg(Vec(n, Bool()))
  val clientDomainIds = Reg(Vec(n, UInt(log2Ceil(nDomains).W)))
  val coreAcquireActive = Wire(Vec(n, Bool()))
  // val coreReleaseActive = Wire(Vec(n, Bool()))
  val doesAccessBank = Seq.fill(n)(Wire(Vec(nBanks, Bool())))
  val throttleReadDomain = RegInit(VecInit(Seq.fill(nDomains)(false.B)))
  // val throttleWriteDomainBanks = RegInit(VecInit(Seq.fill(nDomains)(VecInit(Seq.fill(nBanks)(false.B)))))

  val perfEnable = RegInit(false.B)
  // It is not required to reset these counters but we keep it for now as it helps to close timing
  //  more easily in PnR
  val aCounters = if ( withMonitor ) Some(Seq.fill(n)(RegInit(VecInit(Seq.fill(nBanks)(0.U(64.W)))))) else None
  val cCounters = if ( withMonitor ) Some(Seq.fill(n)(RegInit(VecInit(Seq.fill(nBanks)(0.U(64.W)))))) else None

  val periodCntrReset = periodCntr >= periodLen
  periodCntr := Mux(periodCntrReset || !enBRUGlobal, 0.U, periodCntr + 1.U)

  // generator loop for domains
  for (i <- 0 until nDomains) {
      // bit vectors for clients that are enabled & access mem in the current cycle & are assigned to domain i & are in accssessing bank j
      val clientAcquireActMasked = (clientDomainIds zip coreAcquireActive).map { case (d, act) => d === i.U && act }
      // val clientReleaseActBankMasked = (clientDomainIds zip (coreReleaseActive zip doesAccessBank)).map { case (d, (act, bank)) => d === i.U && act && bank(j) }

      // should be able to reduce or the masks as sytem bus only allows one request per cycle
      val shouldIncAcquire = clientAcquireActMasked.reduce(_||_)
      val nextCntAcquire = Mux(enBRUGlobal, Mux(!periodCntrReset, readCntrs(i) + shouldIncAcquire, shouldIncAcquire), 0.U)

      readCntrs(i) := nextCntAcquire
      throttleReadDomain(i) := Mux(!enBRUGlobal || periodCntrReset, false.B, nextCntAcquire >= maxReads(i))

      // val shouldIncRelease = clientReleaseActBankMasked.reduce(_||_)
      // val nextCntRelease = Mux(enBRUGlobal, Mux(!periodCntrReset, bankWriteCntrs(i)(j) + shouldIncRelease, shouldIncRelease), 0.U)

      // bankWriteCntrs(i)(j) := nextCntRelease
      // throttleWriteDomainBanks(i)(j) := Mux(!enBRUGlobal || periodCntrReset, false.B, nextCntRelease >= maxWrites(i))
  }

  //generator loop for client edges
  for (i <- 0 until n) {
    val (out, edge_out) = outer.adapterNode.out(i)
    val (in, edge_in) = outer.adapterNode.in(i)

    val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
    val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
    val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

    val aIsRead = aIsAcquire || (aIsInstFetch && countInstFetch)

    coreAcquireActive(i) := clientRegEnable(i) && out.a.fire && aIsRead
    //coreReleaseActive(i) := clientRegEnable(i) && edge_out.done(out.c) && cIsWb..

    //per bank support
    //do we access bank j
    val bankBits = Wire(UInt(nBanks.W))
    bankBits := in.a.bits.address(6+numBankBits-1, 6) // Can we make 6 (cache line boundary) not a magic number?
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
        //throttleIO(i).nThrottle(j) := throttleWriteDomainBanks(clientDomainIds(i))(j) && clientRegEnable(i) && enBRUGlobal
        throttleIO(i).nThrottle(j) := false.B
    }

    when (enBRUGlobal && clientRegEnable(i)) {
        when ( throttleReadDomain(clientDomainIds(i)) && aIsRead ) {
           out.a.valid := false.B
           in.a.ready := false.B
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

    // val maxWritesRegField = maxWrites.zipWithIndex.map { case (register, i) =>
    //     (24 + nDomains * 8 + i * 8) -> Seq(RegField(register.getWidth, register,
    //         RegFieldDesc(s"maxWrite$i", s"Max writes for domain $i")))}

    val clientRegEnableRegField = Seq((24 + 2 * nDomains * 8) -> clientRegEnable.zipWithIndex.map { case (client, i) =>
        RegField(client.getWidth, client, RegFieldDesc(s"client${i}En", s"Reg enable for client$i")) })

    val domainIdField = clientDomainIds.zipWithIndex.map { case(domain, i) =>
        (24 + 2 * nDomains * 8 + 8 + i * 8) -> Seq(RegField(domain.getWidth, domain, RegFieldDesc(s"domainId$i", s"Client $i domain ID"))) }

  val perfEnField = Seq((24 + 2 * nDomains * 8 + 8 + n * 8) -> Seq(
    RegField(perfEnable.getWidth, perfEnable,
      RegFieldDesc("perfEnable", "perfEnable"))))

  val mmreg = enBRUGlobalRegField ++ settingsRegField ++ periodLenRegField ++ maxReadsRegField ++ clientRegEnableRegField ++
      domainIdField ++ perfEnField

  (aCounters, cCounters) match {
    case (Some(aCounts), Some(cCounts)) => {
      val bankReadCountersField = aCounts.zipWithIndex.flatMap { case (banks, i) =>
        banks.zipWithIndex.map { case (bank, j) =>
          val addr = 8 * (5 + 2 * nDomains + n + i * nBanks + j)
          addr -> Seq(
            RegField(bank.getWidth, bank,
              RegFieldDesc(s"bankCountR${i * nBanks + j}", s"Bank Read counter"))
          )
        }
      }

      val bankWriteCountersField = cCounts.zipWithIndex.flatMap { case (banks, i) =>
        banks.zipWithIndex.map { case (bank, j) =>
          val addr = 8 * (5 + 2 * nDomains + n + n * nBanks + i * nBanks + j)
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

class WithBRU(address: BigInt = 0x20000000L, nDomains: Int = 4, withMonitor: Boolean = false) extends Config((_, _, _) => {
  case BRUKey => Some(BRUParams(address = address, nDomains = nDomains, withMonitor = withMonitor))
})

// MMIO map examples (nDomains=4, n=4, base 0x00, 8-byte stride):
// Core regs:
//   enBRUGlobal=0x00, countInstrFetch=0x08, periodLen=0x10
//   maxRead[0..3]=0x18..0x30, maxWrite[0..3]=0x38..0x50
//   clientRegEnable=0x58, domainId[0..3]=0x60..0x78, perfEnable=0x80
// Counters (withMonitor=true):
//   nBanks=2: bankCountR[0..7]=0x88..0xC0, bankCountW[0..7]=0xC8..0x100
//   nBanks=4: bankCountR[0..15]=0x88..0x100, bankCountW[0..15]=0x108..0x180
