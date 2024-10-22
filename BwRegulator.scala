package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLBundleA
import freechips.rocketchip.regmapper._
import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy.BufferParams.flow

case class BRUParams (
  address: BigInt,
  nDomains: Int,
  nBanks: Int,
  bankMask: Int,
  withMonitor: Boolean  // avoid including this when using multiple mempress, too many edges and the regmap gets too large
)

case object LLCBRUKey extends Field[Option[BRUParams]](None)
case object DRAMBRUKey extends Field[Option[BRUParams]](None)

class BRUIO(val n: Int) extends Bundle {
  val nThrottleWb = Output(Vec(n, Bool()))
}

class BRUMemIO(val nDomains: Int) extends Bundle {
  val nThrottle = Output(Vec(nDomains, Bool()))
}

class MemRegulator(params: BRUParams)(implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bru-reg-dram",Seq("ku-csl,bru-reg-dram"))

  val node = TLAdapterNode()

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(params.address, 0x7ff)),
    device = device,
    beatBytes = 8)
  
  //val ioNode = BundleBridgeSink[BRUMemIO](Some(() => Flipped(new BRUMemIO(params.nDomains))))
  lazy val module = new MemRegulatorModule(this, params)
}

// Module that sits before the LLC, tags domain id onto TL requests
class MemRegulatorModule(outer: MemRegulator, params: BRUParams) extends LazyModuleImp(outer)
{
  val nClients = outer.node.in.length
  require(params.nDomains <= 32)

  // val memBase = p(ExtMem).get.master.base.U
  var clientNames = new Array[String](nClients)

  val domainIds = Reg(Vec(nClients, UInt(log2Ceil(params.nDomains).W)))

  // No domain generator, handled in counter module

  // Generator for client edges
  for ( i <- 0 until nClients ) {
    val (out, _) = outer.node.out(i) // Wildcard prevents unused warning
    val (in, in_edge) = outer.node.in(i)

    out <> in
    out.a.bits.domainId := domainIds(i)
    out.c.bits.domainId := domainIds(i)

    when ( out.a.fire ) {
      SynthesizePrintf(printf(s"ChanA Core %d, opcode %d, address %x\n", i.U, out.a.bits.opcode, out.a.bits.address))
    }

    when ( out.c.fire ) {
      SynthesizePrintf(printf(s"ChanC Core %d, opcode %d, address %x\n", i.U, out.c.bits.opcode, out.c.bits.address))
    }

    // only support cores for the moment
    clientNames(i) = in_edge.client.clients(0).name
  }

  val domainIdFields = domainIds.zipWithIndex.map { case (reg, i) => 
    (0 + i * 8) -> Seq(RegField(reg.getWidth, reg, 
      RegFieldDesc(s"domainId$i", s"Domain ID for ${clientNames(i)}"))) }

  outer.regnode.regmap(domainIdFields: _*)  
}

class MemCounter(params: BRUParams)(implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bru-count-dram",Seq("ku-csl,bru-count-dram"))

  val node = TLAdapterNode()

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(params.address, 0x7ff)),
    device = device,
    beatBytes = 8)
  
  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {

    val nClients = node.in.length
    println(s"Number of edges into DRAM Counter: $nClients ${node.in}")
    require(nClients >= 1)

    val (out, out_edge) = node.out(0)
    val (in, in_edge) = node.in(0)
    val outParams = out_edge.bundle
    val inParams = in_edge.bundle

    // last connect semantics should work for us here
    out <> in

    val memBase = p(ExtMem).get.master.base.U
    val wPeriod = 25 // for max 33.5ms period, F = 1GHz
    val w = wPeriod - 3 // it can count up to a transaction per 8 cycles when window size is set to max
    // val clientNames = new Array[String](nClients)

    val enGlobal = RegInit(false.B) // CHANGE TO FALSE BEFORE SYNTHESIS
    // val countInstFetch = RegInit(true.B)
    val periodCntr = Reg(UInt(wPeriod.W))
    val periodLen = Reg(UInt(wPeriod.W))
    val readCntrs = Reg(Vec(params.nDomains, UInt(w.W)))
    val maxReads = Reg(Vec(params.nDomains, UInt(w.W)))

    val enDomain = Reg(Vec(params.nDomains, Bool()))
    val domainAcquireActive = Wire(Vec(params.nDomains, Bool()))

    val queues = Seq.fill(params.nDomains)(Module(new Queue(new TLBundleA(inParams), 24, flow=true)))
    val domainArbiter = Module(new RRArbiter(new TLBundleA(inParams), params.nDomains))
    //val bypassArbiter = Module(new Arbiter(new TLBundleA(inParams), 2))

    val periodCntrReset = periodCntr >= periodLen
    periodCntr := Mux(periodCntrReset || !enGlobal, 0.U, periodCntr + 1.U)

    // set some stuff for metasim debug, REMOVE BEFORE SYNTHESIS
    // enDomain(0.U) := true.B
    // maxReads(0.U) := 3.U
    // enDomain(1.U) := true.B
    // maxReads(1.U) := 3.U
    // periodLen := 200.U

    // arbitrate between bypass (unregulated) and queues (regulated)
    // standard arbiter gives priority to lower index
    //bypassArbiter.io.in(1.U) <> domainArbiter.io.out

    // bypass the queues if regulation isn't enabled for the domain
    //bypassArbiter.io.in(0.U).valid := in.a.valid && !(enGlobal && enDomain(in.a.bits.domainId))
    //bypassArbiter.io.in(0.U).bits := in.a.bits

    val selectedQueue = MuxLookup(in.a.bits.domainId, queues(0).io.enq.ready, (0 until params.nDomains).map(i => i.U -> queues(i).io.enq.ready))
    //val isRegulated = enGlobal && enDomain(in.a.bits.domainId)
    in.a.ready := selectedQueue
    //in.a.ready := (bypassArbiter.io.in(0.U).ready && !isRegulated) || (selectedQueue && isRegulated)

    // when ( in.a.fire && !enDomain(in.a.bits.domainId) ) {
    //   printf(s"Bypass domainID %d %d\n", in.a.bits.domainId, bypassArbiter.io.out.fire)
    // }

    val lockDomain = RegInit(false.B)
    val beatingDomain = RegInit(params.nDomains.U)
    val (a_first, a_last, a_done) = out_edge.firstlast(domainArbiter.io.out)

    // first can be high even if we don't fire, make sure we fire
    when ( a_first && !a_last && domainArbiter.io.out.fire ) {
      lockDomain := true.B
      beatingDomain := domainArbiter.io.out.bits.domainId
      //SynthesizePrintf(printf(s"Domain %d locks\n", domainArbiter.io.out.bits.domainId))
    }

    when ( a_done ) {
      lockDomain := false.B
      //SynthesizePrintf(printf(s"Domain %d un-locks\n", domainArbiter.io.out.bits.domainId))
    }

    for ( domain <- 0 until params.nDomains ) {
      // when ( in.a.fire && (in.a.bits.domainId === domain.U) && enDomain(in.a.bits.domainId) ) {
      //   printf(s"Enq to queue %d\n", in.a.bits.domainId)
      // }

      // this fills up fast because of multi-beat
      //assert(queues(domain).io.count =/= 24.U)

      // when regulation enabled for domain, send request to correct queue
      // otherwise we bypass the queues
      queues(domain).io.enq.valid := in.a.valid && (in.a.bits.domainId === domain.U)
      queues(domain).io.enq.bits := in.a.bits

      domainArbiter.io.in(domain) <> queues(domain).io.deq

      domainAcquireActive(domain) := domainArbiter.io.in(domain).fire && domainArbiter.io.in(domain).bits.opcode === TLMessages.Get

      when ( queues(domain).io.deq.fire ) {
        SynthesizePrintf(printf(s"Deq domainID %d, opcode %d, source %d\n", queues(domain).io.deq.bits.domainId, 
                                queues(domain).io.deq.bits.opcode, queues(domain).io.deq.bits.source))
      }

      readCntrs(domain) := Mux(enGlobal, domainAcquireActive(domain) + Mux(periodCntrReset, 0.U, readCntrs(domain)), 0.U)

      when ( lockDomain && ( beatingDomain =/= domain.U ) ) {
        //SynthesizePrintf(printf(s"Locked domain %d, active domain %d\n", domain.U, beatingDomain))
        queues(domain).io.deq.ready := false.B
        domainArbiter.io.in(domain).valid := false.B
      }

      when ( enGlobal && enDomain(domain) ) {
        when ( readCntrs(domain) >= maxReads(domain) ) {
          SynthesizePrintf(printf(s"Throttle domain %d\n", domain.U))
          queues(domain).io.deq.ready := false.B
          domainArbiter.io.in(domain).valid := false.B
        }
      }
    }

    //out.a <> bypassArbiter.io.out
    out.a <> domainArbiter.io.out

    // when ( domainArbiter.io.out.fire && out.a.fire ) {
    //   SynthesizePrintf(printf(s"Deq domainID %d, opcode %d\n", out.a.bits.domainId, out.a.bits.opcode))
    // }

    val enGlobalField = Seq(0 -> Seq(
      RegField(enGlobal.getWidth, enGlobal, RegFieldDesc("enGlobal", "Global Enable"))))

    val periodLenRegField = Seq(8 -> Seq(
      RegField(periodLen.getWidth, periodLen,
        RegFieldDesc("periodLen", "Period length"))))
    
    val maxReadRegFields = maxReads.zipWithIndex.map { case (reg, i) =>
      (16 + i * 8) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"maxAcc$i", s"Maximum access for domain $i"))) }

    val enDomainRegFields = enDomain.zipWithIndex.map { case (reg, i) =>
      (48 + i * 8) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"enDomain$i", s"Per-domian reg enable $i"))) }

    regnode.regmap(enGlobalField ++ periodLenRegField ++
      maxReadRegFields ++ enDomainRegFields: _*)

    // println(s"DRAM Reg Clients:")
    // clientNames.foreach( str => println(s"Client: ${str}"))
  }
}

// BwRegulator = LLC regulation
class BwRegulator(params: BRUParams, location: String) (implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bru-"+location,Seq("ku-csl,bru-"+location))

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(params.address, 0x7ff)),
    device = device,
    beatBytes = 8)

  val node = TLAdapterNode()
  lazy val module = new BwRegulatorModule(this, params)
}

class BwRegulatorModule(outer: BwRegulator, params: BRUParams) extends LazyModuleImp(outer)
{
  // A TLAdapterNode has equal number of input and output edges
  val n = outer.node.in.length
  println(s"Number of edges into BRU: $n")
  require(params.nDomains <= 32) //Limit for regmapper addresses

  val io = IO(new BRUIO(n))

  val memBase = p(ExtMem).get.master.base.U
  val wPeriod = 25 // for max 33.5ms period, F = 1GHz
  val w = wPeriod - 3 // it can count up to a transaction per 8 cycles when window size is set to max
  var clientNames = new Array[String](n)

  val nBanks = p(BankedL2Key).nBanks
  val bankMask = if ( nBanks != 0 ) { ( nBanks - 1 ) << 6 } else { 0 }

  val enBRUGlobal = RegInit(false.B)
  val countInstFetch = RegInit(true.B)
  val countPuts = RegInit(true.B)
  val enWbThrottle = RegInit(false.B)
  val periodCntr = Reg(UInt(wPeriod.W))
  val periodLen = Reg(UInt(wPeriod.W))
  val bankReadCntrs = Seq.fill(params.nDomains)(RegInit(VecInit(Seq.fill(nBanks)(0.U(w.W)))))
  val maxAccs = Reg(Vec(params.nDomains, UInt(w.W)))
  val bankWriteCntrs = Seq.fill(params.nDomains)(RegInit(VecInit(Seq.fill(nBanks)(0.U(w.W)))))
  val maxPuts = Reg(Vec(params.nDomains, UInt(w.W)))
  val wbCntrs = Reg(Vec(params.nDomains, UInt(w.W)))
  val maxWbs = Reg(Vec(params.nDomains, UInt(w.W)))
  val bwREnables = Reg(Vec(n, Bool()))
  val domainIds = Reg(Vec(n, UInt(log2Ceil(params.nDomains).W)))
  val coreAcquireActive = Wire(Vec(n, Bool()))
  val corePutActive = Wire(Vec(n, Bool()))
  val coreWbActive = Wire(Vec(n, Bool()))
  val doesAccessBank = Seq.fill(n)(Wire(Vec(nBanks, Bool())))
  val throttleReadDomainBanks = VecInit(Seq.fill(params.nDomains)(VecInit(Seq.fill(nBanks)(WireInit(Bool(), false.B)))))
  val throttleWriteDomainBanks = VecInit(Seq.fill(params.nDomains)(VecInit(Seq.fill(nBanks)(WireInit(Bool(), false.B)))))

  val throttleDomainWb = Wire(Vec(params.nDomains, Bool()))

  val perfCycleW = 40 // about 8 minutes in target machine time
  val perfPeriodW = 30 // max 100us
  val perfCntrW = perfPeriodW - 3

  val perfEnable = RegInit(false.B)
  val perfPeriod = Reg(UInt(perfPeriodW.W))
  val perfPeriodCntr = Reg(UInt(perfPeriodW.W))
  // It is not required to reset these counters but we keep it for now as it helps to close timing
  //  more easily in PnR
  val aCounters = if ( params.withMonitor ) Some(Seq.fill(n)(RegInit(VecInit(Seq.fill(nBanks)(0.U(64.W)))))) else None
  val cCounters = if ( params.withMonitor ) Some(Seq.fill(n)(RegInit(VecInit(Seq.fill(nBanks)(0.U(64.W)))))) else None
  val cycle = RegInit(0.U(perfCycleW.W))

  cycle := cycle + 1.U
  val perfPeriodCntrReset = perfPeriodCntr >= perfPeriod
  perfPeriodCntr := Mux(perfPeriodCntrReset || !perfEnable, 0.U, perfPeriodCntr + 1.U)

  val periodCntrReset = periodCntr >= periodLen
  periodCntr := Mux(periodCntrReset || !enBRUGlobal, 0.U, periodCntr + 1.U)

  // generator loop for domains
  for (i <- 0 until params.nDomains) {

    for (j <- 0 until nBanks) {
      // bit vectors for clients that are enabled & access mem in the current cycle & are assigned to domain i & are in accssessing bank j
      val clientAcquireActBankMasked = (domainIds zip (coreAcquireActive zip doesAccessBank)).map { case (d, (act, bank)) => d === i.U && act && bank(j) }
      val clientPutActBankMasked = (domainIds zip (corePutActive zip doesAccessBank)).map { case (d, (act, bank)) => d === i.U && act && bank(j) }

      // should be able to reduce or the masks as sytem bus only allows one request per cycle
      bankReadCntrs(i)(j) := Mux(enBRUGlobal, clientAcquireActBankMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, bankReadCntrs(i)(j)), 0.U)
      bankWriteCntrs(i)(j) := Mux(enBRUGlobal, clientPutActBankMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, bankWriteCntrs(i)(j)), 0.U)

      throttleReadDomainBanks(i)(j) := bankReadCntrs(i)(j) >= maxAccs(i)
      throttleWriteDomainBanks(i)(j) := bankWriteCntrs(i)(j) >= maxPuts(i)
    }

    // TODO: get wb throttling to work in Boom
    // leaving this here to legacy, doesn't currently do anything
    val coreWbActMasked = (domainIds zip coreWbActive).map { case (d, act) => d === i.U && act }
    wbCntrs(i) := Mux(enBRUGlobal, coreWbActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, wbCntrs(i)), 0.U)
    throttleDomainWb(i) := wbCntrs(i) >= maxWbs(i)
  }

  //generator loop for client edges
  for (i <- 0 until n) {
    val (out, edge_out) = outer.node.out(i)
    val (in, edge_in) = outer.node.in(i)

    val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
    val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
    // The only Put clients seem to bo RoCC (Mempress) in our setup, writes
    val aIsPut = ( (in.a.bits.opcode === TLMessages.PutFullData) || (in.a.bits.opcode === TLMessages.PutPartialData) ) && in.a.bits.address >= memBase
    // ReleaseData or ProbeAckData cause a PutFull in Broadcast Hub
    val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

    val aIsRead = aIsAcquire || (aIsInstFetch && countInstFetch)

    coreAcquireActive(i) := bwREnables(i) && out.a.fire && aIsRead
    corePutActive(i) := bwREnables(i) && out.a.fire && ( aIsPut && countPuts )
    coreWbActive(i) := bwREnables(i) && edge_out.done(out.c) && cIsWb

    //per bank support
    //do we access bank j
    for (j <- 0 until nBanks) {
      doesAccessBank(i)(j) := bankIndexHelper(in.a.bits.address, bankMask.U) === j.U

      aCounters match {
        case None => // nothing
        case Some(aCounts) => aCounts(i)(j) := Mux(perfEnable, 
                        ((out.a.fire) && (aIsRead || aIsPut) && doesAccessBank(i)(j)) + aCounts(i)(j), 0.U)
      }
      
      cCounters match {
        case None => // nothing
        case Some(cCounts) => cCounts(i)(j) := Mux(perfEnable,
                        ((edge_out.done(out.c) && cIsWb) && doesAccessBank(i)(j)) + cCounts(i)(j), 0.U)
      }

    }

    def bankIndexHelper(address: UInt, mask: UInt): UInt = {
      if ( bankMask != 0 ) {
        //Get bit positions in mask
        def bankBits = (0 until mask.getWidth).filter( i => (mask.litValue & ( 1L << i )) != 0 )
        //Index address with those bit positions
        def bank = bankBits.map(address(_))
        //Convert Bool seq to Vec to UInt
        VecInit(bank).asUInt
      } else {
        0.U
      }
    }

    out <> in
    out.a.bits.domainId := domainIds(i)
    io.nThrottleWb(i) := false.B

    when (enBRUGlobal && bwREnables(i)) {
      for (j <- 0 until nBanks ) {
        when ( ( throttleReadDomainBanks(domainIds(i))(j) && doesAccessBank(i)(j) ) && aIsRead ) {
          out.a.valid := false.B
          in.a.ready := false.B
        }
        when ( ( throttleWriteDomainBanks(domainIds(i))(j) && doesAccessBank(i)(j) ) && aIsPut ) {
          out.a.valid := false.B
          in.a.ready := false.B
        }
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
  }

  val enBRUGlobalRegField = Seq(0 -> Seq(
    RegField(enBRUGlobal.getWidth, enBRUGlobal,
      RegFieldDesc("enBRUGlobal", "Enable BRU Global"))))

  val settingsRegField = Seq(8 -> Seq(
    RegField(countInstFetch.getWidth, countInstFetch,
      RegFieldDesc("countInstFetch", "Count instruction fetch")),
    RegField(enWbThrottle.getWidth, enWbThrottle,
      RegFieldDesc("enWbThrottle", "Enable writeback throttling")),
    RegField(countPuts.getWidth, enWbThrottle,
      RegFieldDesc("countPuts", "Count putFull/putPartial"))))

  val periodLenRegField = Seq(16 -> Seq(
    RegField(periodLen.getWidth, periodLen,
      RegFieldDesc("periodLen", "Period length"))))

  val maxAccRegFields = maxAccs.zipWithIndex.map { case (reg, i) =>
    (24 + i * 8) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"maxAcc$i", s"Maximum access for domain $i"))) }

  val maxPutRegFields = maxPuts.zipWithIndex.map { case (reg, i) =>
    (24 + 8 * params.nDomains + i * 8 ) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"maxPut$i", s"Maximum puts for domain $i"))) }

  val maxWbRegFields = maxWbs.zipWithIndex.map { case (reg, i) =>
    (24 + 2 * 8 * params.nDomains + i * 8) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"maxWb$i", s"Maximum writeback for domain $i"))) }

  val bwREnablesField = Seq((24 + 3 * 8 * params.nDomains) -> bwREnables.zipWithIndex.map { case (bit, i) =>
    RegField(bit.getWidth, bit, RegFieldDesc("bwREnables", s"Enable bandwidth regulation for ${clientNames(i)}")) })

  val domainIdFields = domainIds.zipWithIndex.map { case (reg, i) =>
    (48 + 3 * 8 * params.nDomains + i * 8) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"domainId$i", s"Domain ID for ${clientNames(i)}"))) }

  val perfEnField = Seq((48 + 3 * 8 * params.nDomains + n * 8) -> Seq(
    RegField(perfEnable.getWidth, perfEnable,
      RegFieldDesc("perfEnable", "perfEnable"))))

  val perfPeriodField = Seq((56 + 3 * 8 * params.nDomains + n * 8) -> Seq(
    RegField(perfPeriod.getWidth, perfPeriod,
      RegFieldDesc("perfPeriod", "perfPeriod"))))

  val mmreg = enBRUGlobalRegField ++ settingsRegField ++ periodLenRegField ++ maxAccRegFields ++ maxPutRegFields ++ maxWbRegFields ++ bwREnablesField ++
      domainIdFields ++ perfEnField ++ perfPeriodField

  (aCounters, cCounters) match {
    case (Some(aCounts), Some(cCounts)) => {
      val bankReadCountersField = aCounts.zipWithIndex.flatMap { case (banks, i) =>
        banks.zipWithIndex.map { case (bank, j) =>
          val addr = 8 * (8 + 3 * params.nDomains + n + i * nBanks + j)
          addr -> Seq(
            RegField(bank.getWidth, bank,
              RegFieldDesc(s"bankCountR${i * nBanks + j}", s"Bank Read counter"))
          )
        }
      }

      val bankWriteCountersField = cCounts.zipWithIndex.flatMap { case (banks, i) =>
        banks.zipWithIndex.map { case (bank, j) =>
          val addr = 8 * (8 + 3 * params.nDomains + n + n * nBanks + i * nBanks + j)
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

trait CanHavePeripheryLLCBRU { this: BaseSubsystem =>
  private val portName = "llc-bru"

  val BwRegulator = p(LLCBRUKey) match {
    case Some(params) => {
      val BwRegulator = LazyModule(new BwRegulator(params, "llc")(p))

      pbus.coupleTo(portName) { 
        BwRegulator.regnode := 
        TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }

      Some(BwRegulator)
    }
    case None => None
  }
}

trait CanHaveMemCount {
  val memcount: Option[MemCounter]
  //val memcountIO: Option[BRUMemIO] = memcount.map(_.module.io)
}

trait CanHavePeripheryDRAMBRU { this: BaseSubsystem =>
  private val portName = "dram-bru"

  val MemRegulator = p(DRAMBRUKey) match {
    case Some(params) => {
      val MemRegulator = LazyModule(new MemRegulator(params)(p))

      pbus.coupleTo(portName) {
        MemRegulator.regnode :=
        TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }

      Some(MemRegulator)
    }
    case None => None
  }
}

class WithBRU(address: BigInt = 0x20000000L, nDomains: Int = 4, nBanks: Int = 2, bankMask: Int = 0x40, withMonitor: Boolean = false) extends Config((_, _, _) => {
  case LLCBRUKey => Some(BRUParams(address = address, nDomains = nDomains, nBanks = nBanks, bankMask = bankMask, withMonitor = withMonitor))
})

class WithDRAMBRU(address: BigInt = 0x20002000L, nDomains: Int = 4, nBanks: Int = 2, bankMask: Int = 0x40, withMonitor: Boolean = false) extends Config((_, _, _) => {
  case DRAMBRUKey => Some(BRUParams(address = address, nDomains = nDomains, nBanks = nBanks, bankMask = bankMask, withMonitor = withMonitor))
})
