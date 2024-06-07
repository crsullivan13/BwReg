package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}

case class BRUParams (
  address: BigInt,
  nDomains: Int,
)

case object BRUKey extends Field[Option[BRUParams]](None)

class BRUIO(val n: Int) extends Bundle {
  val nThrottleWb = Output(Vec(n, Bool()))
}

class BwRegulator(params: BRUParams) (implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bru",Seq("ku-csl,bru"))

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(params.address, 0x7ff)),
    device = device,
    beatBytes = 8)

  val node = TLAdapterNode()
  lazy val module = new BwRegulatorModule(this, params.nDomains)
}

class BwRegulatorModule(outer: BwRegulator, nDomains: Int) extends LazyModuleImp(outer)
{
  // A TLAdapterNode has equal number of input and output edges
  val n = outer.node.in.length
  println(s"Number of edges into BRU: $n")
  require(nDomains <= 32)

  val io = IO(new BRUIO(n))

  val memBase = p(ExtMem).get.master.base.U
  val wPeriod = 25 // for max 10ms period, F = 2.13GHz
  val w = wPeriod - 3 // it can count up to a transaction per 8 cycles when window size is set to max
  var clientNames = new Array[String](n)

  val enBRUGlobal = RegInit(false.B)
  val countInstFetch = RegInit(true.B)
  val countPuts = RegInit(true.B)
  val enWbThrottle = RegInit(false.B)
  val periodCntr = Reg(UInt(wPeriod.W))
  val periodLen = Reg(UInt(wPeriod.W))
  val accCntrs = Reg(Vec(nDomains, UInt(w.W)))
  val maxAccs = Reg(Vec(nDomains, UInt(w.W)))
  val putCntrs = Reg(Vec(nDomains, UInt(w.W)))
  val maxPuts = Reg(Vec(nDomains, UInt(w.W)))
  val wbCntrs = Reg(Vec(nDomains, UInt(w.W)))
  val maxWbs = Reg(Vec(nDomains, UInt(w.W)))
  val bwREnables = Reg(Vec(n, Bool()))
  val domainIds = Reg(Vec(n, UInt(log2Ceil(nDomains).W)))
  val coreAcquireActive = Wire(Vec(n, Bool()))
  val corePutActive = Wire(Vec(n,Bool()))
  val coreWbActive = Wire(Vec(n, Bool()))
  val throttleReadDomain = Wire(Vec(nDomains, Bool()))
  val throttleWriteDomain = Wire(Vec(nDomains, Bool()))
  val throttleDomainWb = Wire(Vec(nDomains, Bool()))

  val perfCycleW = 40 // about 8 minutes in target machine time
  val perfPeriodW = 18 // max 100us
  val perfCntrW = perfPeriodW - 3

  val perfEnable = RegInit(false.B)
  val perfPeriod = Reg(UInt(perfPeriodW.W))
  val perfPeriodCntr = Reg(UInt(perfPeriodW.W))
  // It is not required to reset these counters but we keep it for now as it helps to close timing
  //  more easily in PnR
  val aCounters = RegInit(VecInit(Seq.fill(n)(0.U(perfCntrW.W))))
  val cCounters = RegInit(VecInit(Seq.fill(n)(0.U(perfCntrW.W))))
  val cycle = RegInit(0.U(perfCycleW.W))

  cycle := cycle + 1.U
  val perfPeriodCntrReset = perfPeriodCntr >= perfPeriod
  perfPeriodCntr := Mux(perfPeriodCntrReset || !perfEnable, 0.U, perfPeriodCntr + 1.U)

  val periodCntrReset = periodCntr >= periodLen
  periodCntr := Mux(periodCntrReset || !enBRUGlobal, 0.U, periodCntr + 1.U)

  // generator loop for domains
  for (i <- 0 until nDomains) {
    // bit vector for cores that are enabled & access mem in the current cycle & are assigned to domain i
    val coreAcquireActMasked = (domainIds zip coreAcquireActive).map { case (d, act) => d === i.U && act }
    val corePutActMasked = (domainIds zip corePutActive).map { case (d, act) => d === i.U && act }
    // sbus accepts transaction from only one core in a cycle, so it's ok to reduce-or the active cores bit vector
    accCntrs(i) := Mux(enBRUGlobal, coreAcquireActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, accCntrs(i)), 0.U)
    putCntrs(i) := Mux(enBRUGlobal, corePutActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, putCntrs(i)), 0.U)
    throttleReadDomain(i) := accCntrs(i) >= maxAccs(i)
    throttleWriteDomain(i) := putCntrs(i) >= maxPuts(i)

    val coreWbActMasked = (domainIds zip coreWbActive).map { case (d, act) => d === i.U && act }
    wbCntrs(i) := Mux(enBRUGlobal, coreWbActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, wbCntrs(i)), 0.U)
    throttleDomainWb(i) := wbCntrs(i) >= maxWbs(i)
  }

  //generator loop for cores
  for (i <- 0 until n) {
    val (out, edge_out) = outer.node.out(i)
    val (in, edge_in) = outer.node.in(i)

    val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
    val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
    val aIsPut = ( ( in.a.bits.opcode === TLMessages.PutFullData ) || ( in.a.bits.opcode === TLMessages.PutPartialData ) ) && in.a.bits.address >= memBase
    // ReleaseData or ProbeAckData cause a PutFull in Broadcast Hub
    val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

    val aIsRead = aIsAcquire || aIsInstFetch && countInstFetch

    coreAcquireActive(i) := bwREnables(i) && out.a.fire && aIsRead
    corePutActive(i) := bwREnables(i) && out.a.fire && ( aIsPut && countPuts )
    coreWbActive(i) := bwREnables(i) && edge_out.done(out.c) && cIsWb

    out <> in
    io.nThrottleWb(i) := false.B

    when (enBRUGlobal && bwREnables(i)) {
      when (throttleReadDomain(domainIds(i)) && aIsRead) {
        out.a.valid := false.B
        in.a.ready := false.B
      }
      when (throttleWriteDomain(domainIds(i)) && aIsPut) {
        out.a.valid := false.B
        in.a.ready := false.B
      }
      when (throttleDomainWb(domainIds(i)) && enWbThrottle) {
        io.nThrottleWb(i) := true.B
      }
    }

    // a bit hacky but it works for now
    if ( edge_in.client.clients.size > 2 ) {
      // DCache and ICache
      clientNames(i) = edge_in.client.clients(0).name + ", " + edge_in.client.clients(2).name
    } else {
      // RoCC or anything else
      clientNames(i) = edge_in.client.clients(0).name
    }

    when (perfPeriodCntrReset && perfEnable) {
      SynthesizePrintf(printf(s"core: %d %d %d %d %x\n", cycle, i.U, aCounters(i), cCounters(i), out.a.bits.address))
    }
    aCounters(i) := Mux(perfEnable,
      (out.a.fire && (aIsAcquire || aIsInstFetch)) + Mux(perfPeriodCntrReset, 0.U, aCounters(i)), 0.U)
    cCounters(i) := Mux(perfEnable,
      (edge_out.done(out.c) && cIsWb) + Mux(perfPeriodCntrReset, 0.U, cCounters(i)), 0.U)
  }

val enBRUGlobalRegField = Seq(0 -> Seq(
  RegField(enBRUGlobal.getWidth, enBRUGlobal,
    RegFieldDesc("enBRUGlobal", "Enable BRU Global"))))

val settingsRegField = Seq(8 -> Seq(
  RegField(countInstFetch.getWidth, countInstFetch,
    RegFieldDesc("countInstFetch", "Count instruction fetch")),
  RegField(enWbThrottle.getWidth, enWbThrottle,
    RegFieldDesc("enWbThrottle", "Enable writeback throttling")),
  RegField(countPuts.getWidth, countPuts,
    RegFieldDesc("countPuts", "Count putFull/putPartial"))))

val periodLenRegField = Seq(16 -> Seq(
  RegField(periodLen.getWidth, periodLen,
    RegFieldDesc("periodLen", "Period length"))))

val maxAccRegFields = maxAccs.zipWithIndex.map { case (reg, i) =>
  (24 + i * 8) -> Seq(RegField(reg.getWidth, reg,
    RegFieldDesc(s"maxAcc$i", s"Maximum access for domain $i"))) }

val maxPutRegFields = maxPuts.zipWithIndex.map { case (reg, i) =>
  (24 + nDomains * 8 + i * 8) -> Seq(RegField(reg.getWidth, reg,
    RegFieldDesc(s"maxPut$i", s"Maximum puts for domain $i"))) }

val maxWbRegFields = maxWbs.zipWithIndex.map { case (reg, i) =>
  (24 + 2 * nDomains * 8 + i * 8) -> Seq(RegField(reg.getWidth, reg,
    RegFieldDesc(s"maxWb$i", s"Maximum writeback for domain $i"))) }

val bwREnablesField = Seq((24 + 3 * nDomains * 8) -> bwREnables.zipWithIndex.map { case (bit, i) =>
  RegField(bit.getWidth, bit, RegFieldDesc("bwREnables", s"Enable bandwidth regulation for ${clientNames(i)}")) })

val domainIdFields = domainIds.zipWithIndex.map { case (reg, i) =>
  (48 + 3 * nDomains * 8 + i * 8) -> Seq(RegField(reg.getWidth, reg,
    RegFieldDesc(s"domainId$i", s"Domain ID for ${clientNames(i)}"))) }

val perfEnField = Seq((48 + 3 * nDomains * 8 + n * 8) -> Seq(
  RegField(perfEnable.getWidth, perfEnable,
    RegFieldDesc("perfEnable", "perfEnable"))))

val perfPeriodField = Seq((56 + 3 * nDomains * 8 + n * 8) -> Seq(
  RegField(perfPeriod.getWidth, perfPeriod,
    RegFieldDesc("perfPeriod", "perfPeriod"))))

  outer.regnode.regmap(enBRUGlobalRegField ++ settingsRegField ++ periodLenRegField ++ maxAccRegFields ++ maxPutRegFields ++ maxWbRegFields ++
    bwREnablesField ++ domainIdFields ++ perfEnField ++ perfPeriodField: _*)

  println("Bandwidth regulation (BRU):")
  for (i <- clientNames.indices)
    println(s"  $i => ${clientNames(i)}")
}

trait CanHavePeripheryBRU { this: BaseSubsystem =>
  private val portName = "bru"

  val BwRegulator = p(BRUKey) match {
    case Some(params) => {
      val BwRegulator = LazyModule(new BwRegulator(params)(p))

      pbus.coupleTo(portName) { 
        BwRegulator.regnode := 
        TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }

      Some(BwRegulator)
    }
    case None => None
  }
}

class WithBRU(address: BigInt = 0x20000000L, nDomains: Int = 4) extends Config((_, _, _) => {
  case BRUKey => Some(BRUParams(address = address, nDomains = nDomains))
})
