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
  require(nDomains <= 32) //Limit for regmapper addresses

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
  val bank0AccCntrs = Reg(Vec(nDomains, UInt(w.W)))
  val bank1AccCntrs = Reg(Vec(nDomains, UInt(w.W)))
  val maxAccs = Reg(Vec(nDomains, UInt(w.W)))
  val bank0PutCntrs = Reg(Vec(nDomains, UInt(w.W)))
  val bank1PutCntrs = Reg(Vec(nDomains, UInt(w.W)))
  val maxPuts = Reg(Vec(nDomains, UInt(w.W)))
  val wbCntrs = Reg(Vec(nDomains, UInt(w.W)))
  val maxWbs = Reg(Vec(nDomains, UInt(w.W)))
  val bwREnables = Reg(Vec(n, Bool()))
  val domainIds = Reg(Vec(n, UInt(log2Ceil(nDomains).W)))
  val coreAcquireActive = Wire(Vec(n, Bool()))
  val corePutActive = Wire(Vec(n, Bool()))
  val coreWbActive = Wire(Vec(n, Bool()))
  val isAccessBank0 = Wire(Vec(n, Bool()))
  val isAccessBank1 = Wire(Vec(n, Bool()))
  val throttleReadDomainBank0 = Wire(Vec(nDomains, Bool())) //throttle for accesses to bank 0
  val throttleReadDomainBank1 = Wire(Vec(nDomains, Bool())) //throttle for accesses to bank 1
  val throttleWriteDomainBank0 = Wire(Vec(nDomains, Bool())) //throttle for accesses to bank 0
  val throttleWriteDomainBank1 = Wire(Vec(nDomains, Bool())) //throttle for accesses to bank 1

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
    // bit vector for cores that are enabled & access mem in the current cycle & are assigned to domain i & are in accssessing a given bank

    //reads
    val coreAcquireActBank0Masked = (domainIds zip (coreAcquireActive zip isAccessBank0)).map { case (d, (act, bank)) => d === i.U && act && bank }
    val coreAcquireActBank1Masked = (domainIds zip (coreAcquireActive zip isAccessBank1)).map { case (d, (act, bank)) => d === i.U && act && bank }

    //writes from Mempress, TODO: does this apply to other RoCC devices?
    val corePutActBank0Masked = (domainIds zip (corePutActive zip isAccessBank0)).map { case (d, (act, bank)) => d === i.U && act && bank }
    val corePutActBank1Masked = (domainIds zip (corePutActive zip isAccessBank1)).map { case (d, (act, bank)) => d === i.U && act && bank }
    
    // sbus accepts transaction from only one core in a cycle, so it's ok to reduce-or the active cores bit vector
    bank0AccCntrs(i) := Mux(enBRUGlobal, coreAcquireActBank0Masked.reduce(_||_) + Mux(periodCntrReset, 0.U, bank0AccCntrs(i)), 0.U)
    bank1AccCntrs(i) := Mux(enBRUGlobal, coreAcquireActBank1Masked.reduce(_||_) + Mux(periodCntrReset, 0.U, bank1AccCntrs(i)), 0.U)

    bank0PutCntrs(i) := Mux(enBRUGlobal, corePutActBank0Masked.reduce(_||_) + Mux(periodCntrReset, 0.U, bank0PutCntrs(i)), 0.U)
    bank1PutCntrs(i) := Mux(enBRUGlobal, corePutActBank1Masked.reduce(_||_) + Mux(periodCntrReset, 0.U, bank1PutCntrs(i)), 0.U)

    throttleReadDomainBank0(i) := bank0AccCntrs(i) >= maxAccs(i)
    throttleReadDomainBank1(i) := bank1AccCntrs(i) >= maxAccs(i)

    throttleWriteDomainBank0(i) := bank0PutCntrs(i) >= maxPuts(i)
    throttleWriteDomainBank1(i) := bank1PutCntrs(i) >= maxPuts(i)

    val coreWbActMasked = (domainIds zip coreWbActive).map { case (d, act) => d === i.U && act }
    wbCntrs(i) := Mux(enBRUGlobal, coreWbActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, wbCntrs(i)), 0.U)
    throttleDomainWb(i) := wbCntrs(i) >= maxWbs(i)

    when (perfPeriodCntrReset && perfEnable) {
      //SynthesizePrintf(printf(s"domain: %d %d %d %d\n", cycle, i.U, bank0AccCntrs(i), bank1AccCntrs(i)))
    }
  }

  //generator loop for client edges
  for (i <- 0 until n) {
    val (out, edge_out) = outer.node.out(i)
    val (in, edge_in) = outer.node.in(i)

    val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
    val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
    val aIsPut = ( (in.a.bits.opcode === TLMessages.PutFullData) || (in.a.bits.opcode === TLMessages.PutPartialData) ) && in.a.bits.address >= memBase
    // ReleaseData or ProbeAckData cause a PutFull in Broadcast Hub
    val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

    val aIsRead = aIsAcquire || (aIsInstFetch && countInstFetch)

    coreAcquireActive(i) := bwREnables(i) && out.a.fire && aIsRead
    corePutActive(i) := bwREnables(i) && out.a.fire && ( aIsPut && countPuts )
    coreWbActive(i) := bwREnables(i) && edge_out.done(out.c) && cIsWb

    //per bank support
    //do we access a given bank?
    val isBank0 = in.a.bits.address(6) === 0.B
    val isBank1 = in.a.bits.address(6) === 1.B

    isAccessBank0(i) := isBank0
    isAccessBank1(i) := isBank1

    out <> in
    io.nThrottleWb(i) := false.B

    when (enBRUGlobal && bwREnables(i)) {
      when ( ( throttleReadDomainBank0(domainIds(i)) && isAccessBank0(i) || throttleReadDomainBank1(domainIds(i)) && isAccessBank1(i) ) && aIsRead ) {
        out.a.valid := false.B
        in.a.ready := false.B
      }
      when ( ( throttleWriteDomainBank0(domainIds(i)) && isAccessBank0(i) || throttleWriteDomainBank1(domainIds(i)) && isAccessBank1(i) ) && aIsPut) {
        out.a.valid := false.B
        in.a.ready := false.B
      }
      when (throttleDomainWb(domainIds(i)) && enWbThrottle) {
        io.nThrottleWb(i) := true.B
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

    when (perfPeriodCntrReset && perfEnable) {
      SynthesizePrintf(printf(s"core: %d %d %d %d %x\n", cycle, i.U, aCounters(i), cCounters(i), out.a.bits.address))
    }
    aCounters(i) := Mux(perfEnable,
      (out.a.fire && (aIsAcquire || aIsInstFetch || aIsPut)) + Mux(perfPeriodCntrReset, 0.U, aCounters(i)), 0.U)
    cCounters(i) := Mux(perfEnable,
      (edge_out.done(out.c) && cIsWb) + Mux(perfPeriodCntrReset, 0.U, cCounters(i)), 0.U)
  }

  val enBRUGlobalRegField = Seq(0 -> Seq(
    RegField(enBRUGlobal.getWidth, enBRUGlobal,
      RegFieldDesc("enBRUGlobal", "Enable BRU Global"))))

  val settingsRegField = Seq(4*1 -> Seq(
    RegField(countInstFetch.getWidth, countInstFetch,
      RegFieldDesc("countInstFetch", "Count instruction fetch")),
    RegField(enWbThrottle.getWidth, enWbThrottle,
      RegFieldDesc("enWbThrottle", "Enable writeback throttling")),
    RegField(countPuts.getWidth, enWbThrottle,
      RegFieldDesc("countPuts", "Count putFull/putPartial"))))

  val periodLenRegField = Seq(4*2 -> Seq(
    RegField(periodLen.getWidth, periodLen,
      RegFieldDesc("periodLen", "Period length"))))

  val maxAccRegFields = maxAccs.zipWithIndex.map { case (reg, i) =>
    4*(3 + i) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"maxAcc$i", s"Maximum access for domain $i"))) }

  val maxPutRegFields = maxPuts.zipWithIndex.map { case (reg, i) =>
    4*(3 + nDomains + i) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"maxPut$i", s"Maximum puts for domain $i"))) }

  val maxWbRegFields = maxWbs.zipWithIndex.map { case (reg, i) =>
    4*(3 + 2*nDomains + i) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"maxWb$i", s"Maximum writeback for domain $i"))) }

  val bwREnablesField = Seq(4*(3 + 3*nDomains) -> bwREnables.zipWithIndex.map { case (bit, i) =>
    RegField(bit.getWidth, bit, RegFieldDesc("bwREnables", s"Enable bandwidth regulation for ${clientNames(i)}")) })

  val domainIdFields = domainIds.zipWithIndex.map { case (reg, i) =>
    4*(6 + 3*nDomains + i) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"domainId$i", s"Domain ID for ${clientNames(i)}"))) }

  val perfEnField = Seq(4*(6 + 3*nDomains + n) -> Seq(
    RegField(perfEnable.getWidth, perfEnable,
      RegFieldDesc("perfEnable", "perfEnable"))))

  val perfPeriodField = Seq(4*(7 + 3*nDomains + n) -> Seq(
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

class WithBRU(address: BigInt = 0x20000000L, nDomainsArg: Int = 4) extends Config((_, _, _) => {
  case BRUKey => Some(BRUParams(address = address, nDomains = nDomainsArg))
})
