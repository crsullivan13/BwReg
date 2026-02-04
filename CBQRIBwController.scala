package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config._
import freechips.rocketchip.diplomacy._

import freechips.rocketchip.regmapper._
import freechips.rocketchip.tilelink._

class CapabilitiesBundle() extends Bundle {
  val ver = UInt(8.W) // [7:0]
  val nbwblks = UInt(16.W) // [23:8]
  val rpfx = Bool() // [24]
  val p = UInt(4.W) // [28:25]
  val reserved_1 = UInt(3.W) // [31:29]
  val mrbwb = UInt(16.W) // [47:32]
  val reserved_2 = UInt(16.W) // [63:48]
}

object BcAllocCtlStatus extends ChiselEnum {
  val OK           = Value(1.U)
  val INVALID_OP   = Value(2.U)
  val INVALID_RCID = Value(3.U)
  val INVALID_AT   = Value(4.U)
  val INVALID_BWB  = Value(5.U) // invalid or unsupported reserved bandwidth block
}

object BcMonCtlStatus extends ChiselEnum {
  val OK               = Value(1.U)
  val INVALID_OP       = Value(2.U)
  val INVALID_MCID     = Value(3.U)
  val INVALID_EVT_ID   = Value(4.U)
  val INVALID_AT       = Value(5.U)
}

object BcMonCtlEvent extends ChiselEnum {
  val NONE             = Value(0.U)
  val READ_WRITE       = Value(1.U)
  val READ_ONLY        = Value(2.U)
  val WRITE_ONLY       = Value(3.U)

  val CUSTOM           = Value(255.U)
}

object BcCtlOp extends ChiselEnum {
  val CONFIG = Value(1.U)
  val READ   = Value(2.U)

  val CUSTOM       = Value(31.U) // force this type to be 5 bits wide, this is dumb
}

class MonCommandResponse(dataBits: Int) extends Bundle {
    val status = BcMonCtlStatus()
    val hasData = Bool()
    val data = UInt(dataBits.W)
}

class AllocCommandResponse(dataBits: Int) extends Bundle {
    val status = BcAllocCtlStatus()
    val hasData = Bool()
    val data = UInt(dataBits.W)
}

class BcMonCommand(nMCID: Int) extends Bundle {
    val op = BcCtlOp()
    val mcid = UInt(log2Ceil(nMCID).W)
    val event = BcMonCtlEvent()
}

class BcAllocCommand(nRCID: Int) extends Bundle {
    val op = BcCtlOp()
    val at = UInt(3.W)
    val rcid = UInt(log2Ceil(nRCID).W)

    val rbwb = UInt(16.W)
}

class CBQRIBwController(device: SimpleDevice, params: BRUParams)(implicit p: Parameters) extends LazyModule()(p) {
    val regnode = new TLRegisterNode(
        address = Seq(AddressSet(params.address, 0x7ff)),
        device = device,
        beatBytes = 8)

    val nRCID = params.nRCID
    val nMCID = params.nMCID
    val nbwblks = params.nbwblks
    val mrbwb = params.mrbwb

    val wPeriod = 25 // for max 33.5ms period, F = 1GHz

    lazy val module = new Impl
    class Impl extends LazyModuleImp(this) {
        val io = IO(new Bundle {
            val bc_mon_command = Valid(new BcMonCommand(nMCID))
            val bc_mon_resp = Flipped(Valid(new MonCommandResponse(62)))

            val bc_alloc_ctl_command = Valid(new BcAllocCommand(nRCID))
            val bc_alloc_ctl_resp = Flipped(Valid(new AllocCommandResponse(16)))

            val enGlobal = Output(Bool())
            val periodLen = Output(UInt(wPeriod.W))
        })

        val enGlobal = RegInit(false.B)
        val periodLen = Reg(UInt(wPeriod.W))
        io.enGlobal := enGlobal
        io.periodLen := periodLen

        val bc_capabilities = WireDefault(0.U.asTypeOf(new CapabilitiesBundle))
        bc_capabilities.ver := params.ver.U
        bc_capabilities.nbwblks := nbwblks.U
        bc_capabilities.rpfx := params.rpfx.B
        bc_capabilities.p := params.p.U
        bc_capabilities.mrbwb := mrbwb.U

        // pack the fields explicitly for regmap
        val bc_capabilities_u64 = Cat(bc_capabilities.reserved_2,
        bc_capabilities.mrbwb,
        bc_capabilities.reserved_1,
        bc_capabilities.p,
        bc_capabilities.rpfx,
        bc_capabilities.nbwblks,
        bc_capabilities.ver)

        val bc_mon_ctl_op = WireDefault(BcCtlOp.READ)          // [4:0]
        val bc_mon_ctl_at = WireDefault(0.U(3.W))              // [7:5]
        val bc_mon_ctl_mcid = WireDefault(0.U(12.W))           // [19:8]
        val bc_mon_ctl_evtid = WireDefault(BcMonCtlEvent.NONE) // [27:20]
        val bc_mon_ctl_atv = WireDefault(false.B)              // [28]
        val bc_mon_ctl_reserved_1 = WireDefault(0.U(3.W))      // [31:29]
        val bc_mon_ctl_status = RegInit(BcMonCtlStatus.OK)     // [38:32]
        val bc_mon_ctl_busy = RegInit(false.B)                 // [39]
        val bc_mon_ctl_reserved_2 = WireDefault(0.U(24.W))     // [63:40]

        val bc_mon_ctl_u64 = Cat(bc_mon_ctl_reserved_2,
        bc_mon_ctl_busy,
        bc_mon_ctl_status.asUInt.pad(7),
        bc_mon_ctl_reserved_1,
        bc_mon_ctl_atv,
        bc_mon_ctl_evtid.asUInt.pad(8),
        bc_mon_ctl_mcid,
        bc_mon_ctl_at,
        bc_mon_ctl_op.asUInt.pad(5))

        io.bc_mon_command.valid := false.B
        io.bc_mon_command.bits.op := BcCtlOp.CUSTOM
        io.bc_mon_command.bits.mcid := 0.U
        io.bc_mon_command.bits.event := BcMonCtlEvent.NONE
        def writeBcMonCtl(valid: Bool, data: UInt): Bool = {
            when ( valid && !bc_mon_ctl_busy ) {
                // bc_mon_ctl_at := data(7,5)
                // bc_mon_ctl_atv := data(28)
                // bc_mon_ctl_busy := true.B
                io.bc_mon_command.bits.op := data(4,0).asTypeOf(BcCtlOp())
                io.bc_mon_command.bits.mcid := data(19,8)
                io.bc_mon_command.bits.event := data(27,20).asTypeOf(BcMonCtlEvent())

                io.bc_mon_command.valid := true.B
            }

            !bc_mon_ctl_busy
        }

        def readBcMonCtl(ready: Bool): (Bool, UInt) = {
            (true.B, bc_mon_ctl_u64(31,0))
        }

        val bc_mon_ctr_val = RegInit(0.U(62.W)) // [61:0]
        val bc_mon_ctr_inv = RegInit(false.B)   // [62]
        val bc_mon_ctr_ovf = RegInit(false.B)   // [63]

        when ( io.bc_mon_resp.valid ) {
            when ( io.bc_mon_resp.bits.hasData ) {
                bc_mon_ctr_val := io.bc_mon_resp.bits.data
            }
            bc_mon_ctl_status := io.bc_mon_resp.bits.status
        }

        val bc_mon_ctr_u64 = Cat(bc_mon_ctr_ovf,
        bc_mon_ctr_inv,
        bc_mon_ctr_val)

        val bc_alloc_ctl_op = WireDefault(BcCtlOp.READ)        // [4:0]
        val bc_alloc_ctl_at = WireDefault(0.U(3.W))            // [7:5]
        val bc_alloc_ctl_rcid = WireDefault(0.U(12.W))         // [19:8]
        val bc_alloc_ctl_reserved_1 = WireDefault(0.U(12.W))   // [31:20] 
        val bc_alloc_ctl_status = RegInit(BcAllocCtlStatus.OK) // [38:32]
        val bc_alloc_ctl_busy = RegInit(false.B)               // [39]
        val bc_alloc_ctl_reserved_2 = WireDefault(0.U(24.W))   // [63:40]

        val bc_alloc_ctl_u64 = Cat(bc_alloc_ctl_reserved_2,
        bc_alloc_ctl_busy,
        bc_alloc_ctl_status.asUInt.pad(7),
        bc_alloc_ctl_reserved_1,
        bc_alloc_ctl_rcid,
        bc_alloc_ctl_at,
        bc_alloc_ctl_op.asUInt.pad(5))

        io.bc_alloc_ctl_command.valid := false.B
        io.bc_alloc_ctl_command.bits.op := BcCtlOp.CUSTOM
        io.bc_alloc_ctl_command.bits.at := 0.U
        io.bc_alloc_ctl_command.bits.rcid := 0.U
        def writeBcAllocCtl(valid: Bool, data: UInt): Bool = {
            when ( valid && !bc_alloc_ctl_busy ) {
                // bc_alloc_ctl_busy := true.B everything should end up being single cycle
                io.bc_alloc_ctl_command.bits.op := data(4,0).asTypeOf(BcCtlOp())
                io.bc_alloc_ctl_command.bits.at := data(7,5)
                io.bc_alloc_ctl_command.bits.rcid := data(19,8)

                io.bc_alloc_ctl_command.valid := true.B
            }

            !bc_alloc_ctl_busy
        }

        def readBcAllocCtlLo32(ready: Bool): (Bool, UInt) = {
            (true.B, bc_alloc_ctl_u64(31,0))
        }

        val bc_bw_alloc_rbwb = RegInit(0.U(16.W))          // [15:0]
        val bc_bw_alloc_reserved_1 = WireDefault(0.U(4.W)) // [19:16]
        val bc_bw_alloc_mweight = RegInit(0.U(18.W))       // [27:20]
        val bc_bw_alloc_shared_at = RegInit(0.U(3.W))      // [30:28]
        val bc_bw_alloc_use_shared = RegInit(false.B)      // [31]
        val bc_bw_alloc_reserved_2 = RegInit(0.U(32.W))    // [63:32]

        io.bc_alloc_ctl_command.bits.rbwb := bc_bw_alloc_rbwb

        when ( io.bc_alloc_ctl_resp.valid ) {
            when ( io.bc_alloc_ctl_resp.bits.hasData ) {
                bc_bw_alloc_rbwb := io.bc_alloc_ctl_resp.bits.data
            }
            bc_alloc_ctl_status := io.bc_alloc_ctl_resp.bits.status
        }

        val bc_bw_alloc_u64 = Cat(bc_bw_alloc_reserved_2,
        bc_bw_alloc_use_shared,
        bc_bw_alloc_shared_at,
        bc_bw_alloc_mweight,
        bc_bw_alloc_reserved_1,
        bc_bw_alloc_rbwb)

        def writeBcBwAlloc(valid: Bool, data: UInt): Bool = {
            when ( valid ) {
                bc_bw_alloc_rbwb := data(15,0)
                // bc_bw_alloc_mweight := data(27,20) ignore these fields for now
                // bc_bw_alloc_shared_at := data(30,28)
                // bc_bw_alloc_use_shared := data(31)
            }

            !bc_alloc_ctl_busy
        }

        def readBcBwAlloc(ready: Bool): (Bool, UInt) = {
            (true.B, bc_bw_alloc_u64(31,0))
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

        val bc_alloc_ctl_lo32 = RegField(32, readBcAllocCtlLo32(_), writeBcAllocCtl(_,_), 
            RegFieldDesc("bc_alloc_ctl_lo32", "Writeable portion of bc_alloc_ctl"))

        val bc_alloc_ctl_hi32 = RegField.r(32, bc_alloc_ctl_u64(63,32),
            RegFieldDesc("bc_alloc_ctl_hi32", "Status and busy bits of bc_alloc_ctl"))

        val bc_bw_alloc_lo32 = RegField(32, readBcBwAlloc(_), writeBcBwAlloc(_,_), 
            RegFieldDesc("bc_bw_alloc_lo32", "Writeable portion of bc_bw_alloc"))

        val bc_bw_alloc_hi32 = RegField.r(32, bc_bw_alloc_u64(63,32),
            RegFieldDesc("bc_bw_alloc_hi32", "Reserved bits of bc_bw_alloc"))

        val global_enable = RegField(enGlobal.getWidth, enGlobal,
            RegFieldDesc("enBRUGlobal", "Custom global regulator enable"))

        val period_length = RegField(periodLen.getWidth, periodLen,
            RegFieldDesc("periodLen", "Custom period length control"))

        val regmap = regnode.regmap(
            0x000 -> RegFieldGroup("bc_capabilities", Some("4.1. Bandwidth-controller Capabilities"), Seq(bc_capabilities_lo32, bc_capabilities_hi32)),
            0x008 -> RegFieldGroup("bc_mon_ctl", Some("4.2. Bandwidth Usage Monitoring Control"), Seq(bc_mon_ctl_lo32, bc_mon_ctl_hi32)),
            0x010 -> RegFieldGroup("bc_mon_ctr", Some("4.3. Bandwidth Monitoring Counter Value"), Seq(bc_mon_ctr_lo32, bc_mon_ctr_hi32)),
            0x018 -> RegFieldGroup("bc_alloc_ctl", Some("4.4. Bandwidth Allocation Control"), Seq(bc_alloc_ctl_lo32, bc_alloc_ctl_hi32)),
            0x020 -> RegFieldGroup("bc_bw_alloc", Some("4.5. Bandwidth Allocation Configuration"), Seq(bc_bw_alloc_lo32, bc_bw_alloc_hi32)),
            0x100 -> Seq(global_enable),
            0x108 -> Seq(period_length)
        )
    }
}