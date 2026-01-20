package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
//import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}

import freechips.rocketchip.tile.{BRUTileIO, BRUTileAccessIO}

class CapabilitiesBundle() extends Bundle {
  val ver = UInt(8.W) // [7:0]
  val nbwblks = UInt(16.W) // [23:8]
  val rpfx = Bool() // [24]
  val p = UInt(4.W) // [28:25]
  val reserved_1 = UInt(3.W) // [31:29]
  val mrbwb = UInt(16.W) // [47:32]
  val reserved_2 = UInt(16.W) // [63:48]
}

case class BRUParams (
  address: BigInt,
  nDomains: Int,
  withMonitor: Boolean,

  // bc_capabilities
  ver: Int,
  nbwblks: Int,
  rpfx: Boolean,
  p: Int,
  mrbwb: Int
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
  // NOTE: we currently assume that nDomains == number of cores
  // if you decide to stray from this, change params.nDomains below to be equal to number of cores
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
  val bankReadCntrs = Seq.fill(nDomains)(RegInit(VecInit(Seq.fill(nBanks)(0.U(w.W)))))
  val maxReads = Reg(Vec(nDomains, UInt(w.W)))
  // val bankWriteCntrs = Seq.fill(nDomains)(RegInit(VecInit(Seq.fill(nBanks)(0.U(w.W)))))
  // val maxWrites = Reg(Vec(nDomains, UInt(w.W)))
  val clientRegEnable = Reg(Vec(n, Bool()))
  val clientDomainIds = Reg(Vec(n, UInt(log2Ceil(nDomains).W)))
  val coreAcquireActive = Wire(Vec(n, Bool()))
  // val coreReleaseActive = Wire(Vec(n, Bool()))
  val doesAccessBank = Seq.fill(n)(Wire(Vec(nBanks, Bool())))
  val throttleReadDomainBanks = RegInit(VecInit(Seq.fill(nDomains)(VecInit(Seq.fill(nBanks)(false.B)))))
  // val throttleWriteDomainBanks = RegInit(VecInit(Seq.fill(nDomains)(VecInit(Seq.fill(nBanks)(false.B)))))

  val bc_capabilities = WireDefault(0.U.asTypeOf(new CapabilitiesBundle))
  bc_capabilities.ver := params.ver.U
  bc_capabilities.nbwblks := params.nbwblks.U
  bc_capabilities.rpfx := params.rpfx.B
  bc_capabilities.p := params.p.U
  bc_capabilities.mrbwb := params.mrbwb.U

  // pack the fields explicitly for regmap
  val bc_capabilities_u64 = Cat(bc_capabilities.reserved_2,
    bc_capabilities.mrbwb,
    bc_capabilities.reserved_1,
    bc_capabilities.p,
    bc_capabilities.rpfx,
    bc_capabilities.nbwblks,
    bc_capabilities.ver)

  val bc_mon_ctl_op = RegInit(0.U(5.W))                 // [4:0]
  val bc_mon_ctl_at = RegInit(0.U(3.W))                 // [7:5]
  val bc_mon_ctl_mcid = RegInit(0.U(12.W))              // [19:8]
  val bc_mon_ctl_evtid = RegInit(0.U(8.W))              // [27:20]
  val bc_mon_ctl_atv = RegInit(false.B)                 // [28]
  val bc_mon_ctl_reserved_1 = WireDefault(0.U(3.W))     // [31:29]
  val bc_mon_ctl_status = RegInit(0.U(7.W))             // [38:32]
  val bc_mon_ctl_busy = RegInit(false.B)                // [39]
  val bc_mon_ctl_reserved_2 = WireDefault(0.U(24.W))    // [63:40]

  val bc_mon_ctl_u64 = Cat(bc_mon_ctl_reserved_2,
    bc_mon_ctl_busy,
    bc_mon_ctl_status,
    bc_mon_ctl_reserved_1,
    bc_mon_ctl_atv,
    bc_mon_ctl_evtid,
    bc_mon_ctl_mcid,
    bc_mon_ctl_at,
    bc_mon_ctl_op)

  def writeBcMonCtl(valid: Bool, data: UInt): Bool = {
    when ( valid && !bc_mon_ctl_busy ) {
      bc_mon_ctl_op := data(4,0)
      bc_mon_ctl_at := data(7,5)
      bc_mon_ctl_mcid := data(19,8)
      bc_mon_ctl_evtid := data(27,20)
      bc_mon_ctl_atv := data(28)
      bc_mon_ctl_busy := true.B
      bc_mon_ctl_status := 0.U 
    }

    !bc_mon_ctl_busy
  }

  def readBcMonCtl(ready: Bool): (Bool, UInt) = {
    (true.B, bc_mon_ctl_u64(31,0))
  }

  val bc_mon_ctr_val = RegInit(0.U(62.W)) // [61:0]
  val bc_mon_ctr_inv = RegInit(false.B)   // [62]
  val bc_mon_ctr_ovf = RegInit(false.B)   // [63]

  val bc_mon_ctr_u64 = Cat(bc_mon_ctr_ovf,
    bc_mon_ctr_inv,
    bc_mon_ctr_val)


  val bc_alloc_ctl_op = RegInit(0.U(5.W))              // [4:0]
  val bc_alloc_ctl_at = RegInit(0.U(3.W))              // [7:5]
  val bc_alloc_ctl_rcid = RegInit(0.U(12.W))           // [19:8]
  val bc_alloc_ctl_reserved_1 = WireDefault(0.U(12.W)) // [31:20] 
  val bc_alloc_ctl_status = RegInit(0.U(7.W))          // [38:32]
  val bc_alloc_ctl_busy = RegInit(false.B)             // [39]
  val bc_alloc_ctl_reserved_2 = WireDefault(0.U(24.W)) // [63:40]

  val bc_alloc_ctl_u64 = Cat(bc_alloc_ctl_reserved_2,
    bc_alloc_ctl_busy,
    bc_alloc_ctl_status,
    bc_alloc_ctl_reserved_1,
    bc_alloc_ctl_rcid,
    bc_alloc_ctl_at,
    bc_alloc_ctl_op)

  def writeBcAllocCtl(valid: Bool, data: UInt): Bool = {
    when ( valid && !bc_alloc_ctl_busy ) {
      bc_alloc_ctl_op := data(4,0)
      bc_alloc_ctl_at := data(7,5)
      bc_alloc_ctl_rcid := data(19,8)
      bc_alloc_ctl_status := 0.U
      bc_alloc_ctl_busy := true.B
    }

    !bc_alloc_ctl_busy
  }

  def readBcAllocCtl(ready: Bool): (Bool, UInt) = {
    (true.B, bc_alloc_ctl_u64(31,0))
  }

  val bc_bw_alloc_rbwb = RegInit(0.U(16.W))          // [15:0]
  val bc_bw_alloc_reserved_1 = WireDefault(0.U(4.W)) // [19:16]
  val bc_bw_alloc_mweight = RegInit(0.U(18.W))       // [27:20]
  val bc_bw_alloc_shared_at = RegInit(0.U(3.W))      // [30:28]
  val bc_bw_alloc_use_shared = RegInit(false.B)      // [31]
  val bc_bw_alloc_reserved_2 = RegInit(0.U(32.W))    // [63:32]

  val bc_bw_alloc_u64 = Cat(bc_bw_alloc_reserved_2,
    bc_bw_alloc_use_shared,
    bc_bw_alloc_shared_at,
    bc_bw_alloc_mweight,
    bc_bw_alloc_reserved_1,
    bc_bw_alloc_rbwb)

  def writeBcBwAlloc(valid: Bool, data: UInt): Bool = {
    when ( valid ) {
      bc_bw_alloc_rbwb := bc_bw_alloc_u64(15,0)
      bc_bw_alloc_mweight := bc_bw_alloc_u64(27,20)
      bc_bw_alloc_shared_at := bc_bw_alloc_u64(30,28)
      bc_bw_alloc_use_shared := bc_bw_alloc_u64(31)
    }

    !bc_alloc_ctl_busy
  }

  def readBcBwAlloc(ready: Bool): (Bool, UInt) = {
    (true.B, bc_bw_alloc_u64(31,0))
  }

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
      // val clientReleaseActBankMasked = (clientDomainIds zip (coreReleaseActive zip doesAccessBank)).map { case (d, (act, bank)) => d === i.U && act && bank(j) }

      // should be able to reduce or the masks as sytem bus only allows one request per cycle
      val shouldIncAcquire = clientAcquireActBankMasked.reduce(_||_)
      val nextCntAcquire = Mux(enBRUGlobal, Mux(!periodCntrReset, bankReadCntrs(i)(j) + shouldIncAcquire, shouldIncAcquire), 0.U)

      bankReadCntrs(i)(j) := nextCntAcquire
      throttleReadDomainBanks(i)(j) := Mux(!enBRUGlobal || periodCntrReset, false.B, nextCntAcquire >= maxReads(i))

      // val shouldIncRelease = clientReleaseActBankMasked.reduce(_||_)
      // val nextCntRelease = Mux(enBRUGlobal, Mux(!periodCntrReset, bankWriteCntrs(i)(j) + shouldIncRelease, shouldIncRelease), 0.U)

      // bankWriteCntrs(i)(j) := nextCntRelease
      // throttleWriteDomainBanks(i)(j) := Mux(!enBRUGlobal || periodCntrReset, false.B, nextCntRelease >= maxWrites(i))
    }
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
    //coreReleaseActive(i) := clientRegEnable(i) && edge_out.done(out.c) && cIsWb

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
      for (j <- 0 until nBanks ) {
        when ( ( throttleReadDomainBanks(clientDomainIds(i))(j) && doesAccessBank(i)(j) ) && aIsRead ) {
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


  val bc_capabilities_lo32 = RegField.r(32, bc_capabilities_u64(31,0),
    RegFieldDesc("bc_capabilities_lo32", "Low 32 bits of bc_capabilities"))
  
  val bc_capabilities_hi32 = RegField.r(32, bc_capabilities_u64(63,32),
    RegFieldDesc("bc_capabilities_hi32", "High 32 bits of bc_capabilities"))

  val bc_mon_ctl_lo32 = RegField(32, readBcMonCtl(_), writeBcMonCtl(_,_), 
    RegFieldDesc("bc_mon_ctl_lo32", "Writeable portion of bc_mon_ctl"))

  val bc_mon_ctl_hi32 = RegField.r(32, bc_mon_ctl_u64(63,32),
    RegFieldDesc("bc_mon_ctl_hi32", "Status and busy bits of bc_mon_ctl"))

  val bc_mon_ctr_lo32 = RegField.r(32, bc_mon_ctr_u64(31,0),
    RegFieldDesc("bc_mon_ctr_lo32", "Low 32 bits of bc_mon_ctr"))

  val bc_mon_ctr_hi32 = RegField.r(32, bc_mon_ctr_u64(63,32),
    RegFieldDesc("bc_mon_ctr_hi32", "High 32 bits of bc_mon_ctr"))

  val bc_alloc_ctl_lo32 = RegField(32, readBcAllocCtl(_), writeBcAllocCtl(_,_), 
    RegFieldDesc("bc_alloc_ctl_lo32", "Writeable portion of bc_alloc_ctl"))

  val bc_alloc_ctl_hi32 = RegField.r(32, bc_alloc_ctl_u64(63,32),
    RegFieldDesc("bc_alloc_ctl_hi32", "Status and busy bits of bc_alloc_ctl"))

  val bc_bw_alloc_lo32 = RegField(32, readBcBwAlloc(_), writeBcBwAlloc(_,_), 
    RegFieldDesc("bc_bw_alloc_lo32", "Writeable portion of bc_bw_alloc"))

  val bc_bw_alloc_hi32 = RegField.r(32, bc_bw_alloc_u64(63,32),
    RegFieldDesc("bc_bw_alloc_hi32", "Reserved bits of bc_bw_alloc"))

  val regmap = outer.regnode.regmap(
    0x000 -> RegFieldGroup("bc_capabilities", Some("4.1. Bandwidth-controller Capabilities"), Seq(bc_capabilities_lo32, bc_capabilities_hi32)),
    0x008 -> RegFieldGroup("bc_mon_ctl", Some("4.2. Bandwidth Usage Monitoring Control"), Seq(bc_mon_ctl_lo32, bc_mon_ctl_hi32)),
    0x010 -> RegFieldGroup("bc_mon_ctr", Some("4.3. Bandwidth Monitoring Counter Value"), Seq(bc_mon_ctr_lo32, bc_mon_ctr_hi32)),
    0x018 -> RegFieldGroup("bc_alloc_ctl", Some("4.4. Bandwidth Allocation Control"), Seq(bc_alloc_ctl_lo32, bc_alloc_ctl_hi32)),
    0x020 -> RegFieldGroup("bc_bw_alloc", Some("4.5. Bandwidth Allocation Configuration"), Seq(bc_bw_alloc_lo32, bc_bw_alloc_hi32))
  )

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

class WithBRU(address: BigInt = 0x20000000L, nDomains: Int = 4, withMonitor: Boolean = false,
             ver: Int = 1, nbwblks: Int = 65536, rpfx: Boolean = false, p: Int = 0, mrbwb: Int = 65536) 
extends Config((_, _, _) => {
  case BRUKey => {
    Some(BRUParams(
      address = address, 
      nDomains = nDomains, 
      withMonitor = withMonitor,

      ver = ver,
      nbwblks = nbwblks,
      rpfx = rpfx,
      p = p,
      mrbwb = mrbwb
    ))
  }
})

// MMIO map examples (nDomains=4, n=4, base 0x00, 8-byte stride):
// Core regs:
//   enBRUGlobal=0x00, countInstrFetch=0x08, periodLen=0x10
//   maxRead[0..3]=0x18..0x30, maxWrite[0..3]=0x38..0x50
//   clientRegEnable=0x58, domainId[0..3]=0x60..0x78, perfEnable=0x80
// Counters (withMonitor=true):
//   nBanks=2: bankCountR[0..7]=0x88..0xC0, bankCountW[0..7]=0xC8..0x100
//   nBanks=4: bankCountR[0..15]=0x88..0x100, bankCountW[0..15]=0x108..0x180
