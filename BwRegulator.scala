package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
//import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}

import freechips.rocketchip.tile.{BRUTileIO, BRUTileAccessIO, BRUPerBankTileIO}

// BRUTileIO defined in BaseTile.scala so we have it everywhere

case class BRUParams (
  address: BigInt,
  nDomains: Int,
  withMonitor: Boolean, // does nothing on this branch
  nDramBanks: Int,
  dramBankOffset: Int
)

case object BRUKey extends Field[Option[BRUParams]](None)

class BwRegulator(params: BRUParams)(implicit p: Parameters) extends LazyModule
{
    val device = new SimpleDevice("bru",Seq("bru"))

    // first number is number of cores, second is number of banks
    // TODO: Can we grab the number of cores from params somehow?
    // for now assume number of cores == nDomains
    val ioNode = Seq.fill(params.nDomains)(BundleBridgeSource(() => new BRUTileIO(p(SubsystemBankedCoherenceKey).nBanks)))
    val dramRegNode = BundleBridgeSink[BRUPerBankTileIO](Some(() => Flipped(new BRUPerBankTileIO(params.nDomains, params.nDramBanks))))
    // val coreAccessNode = Seq.fill(4)(BundleBridgeSink[BRUTileAccessIO](Some(() => Flipped(new BRUTileAccessIO(p(SubsystemBankedCoherenceKey).nBanks)))))
    val adapterNode = TLAdapterNode()

    // add simple config registers
    val regnode = new TLRegisterNode(
        address = Seq(AddressSet(params.address, 0x7ff)),
        device = device,
        beatBytes = 8)

    lazy val module = new BwRegulatorModule(this, params)
}

class BwRegulatorModule(outer: BwRegulator, params: BRUParams) extends LazyModuleImp(outer)
{
  val throttleIO = outer.ioNode.map(_.bundle)

  val nDomains = params.nDomains
  val numDramBanks = params.nDramBanks
  val numCacheBanks = p(SubsystemBankedCoherenceKey).nBanks
  val dramBankOffset = params.dramBankOffset
  val dramBankMask = numDramBanks - 1

  val memBase = p(ExtMem).get.master.base.U

  val adapterNode = outer.adapterNode
  val nClients = adapterNode.in.length
  println(s"Number of edges into BRU: $nClients")

  val globalEnable = RegInit(false.B)

  val clientRegEnable = Reg(Vec(nClients, Bool()))
  val clientDomainIds = Reg(Vec(nClients, UInt(log2Ceil(nDomains).W))) // which domain is a client in

  val doesClientFireAcquire = Wire(Vec(nClients, Bool()))
  val doesClientAccessBank = Seq.fill(nClients)(Wire(Vec(numDramBanks, Bool())))

  for ( i <- 0 until nDomains ) {
    for ( j <- 0 until numDramBanks ) {
        val clientDomainActive = ( clientDomainIds zip ( doesClientFireAcquire zip doesClientAccessBank ) ).map {
            case (domain, (active, bank)) => domain === i.U && active && bank(j)
        }
    }
  }

  for ( i <- 0 until nClients ) {
      val (out, edge_out) = adapterNode.out(i)
      val (in, edge_in) = adapterNode.in(i)

      out <> in

      val isAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
      val isInstrFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase

      val isAccessRead = isAcquire || isInstrFetch

      doesClientFireAcquire(i) := isAccessRead && in.a.fire && clientRegEnable(i)

      out.a.bits.domainId := clientDomainIds(i)
      out.c.bits.domainId := clientDomainIds(i)

      for ( j <- 0 until numDramBanks ) {
        doesClientAccessBank(i)(j) := ( ( in.a.bits.address >> dramBankOffset.U ) & dramBankMask.U ) === j.U
      }

      for ( j <- 0 until numCacheBanks ) {
        throttleIO(i).nThrottle(j) := false.B
      }

      val domainThrottle = outer.dramRegNode.bundle.nThrottle(clientDomainIds(i))
      when ( clientRegEnable(i) && globalEnable ) {
          for ( j <- 0 until numDramBanks ) {
              when ( doesClientAccessBank(i)(j) && isAccessRead && domainThrottle(j) ) {
                  in.a.ready := false.B
                  out.a.valid := false.B
              }
          }
      }
    }

    val globalEnableRegField = Seq(0 -> Seq(
        RegField(globalEnable.getWidth, globalEnable,
            RegFieldDesc("globalEnable", "Toggle entire unit"))))

    val clientRegEnableRegField = Seq((16) -> clientRegEnable.zipWithIndex.map { case (client, i) =>
        RegField(client.getWidth, client, RegFieldDesc(s"client${i}En", s"Reg enable for client$i")) })

    val domainIdField = clientDomainIds.zipWithIndex.map { case(domain, i) =>
        (32 + nDomains * 8 + i * 8) -> Seq(RegField(domain.getWidth, domain, RegFieldDesc(s"domainId$i", s"Client $i domain ID"))) }

    val mmioReg = globalEnableRegField ++ clientRegEnableRegField ++ domainIdField

    outer.regnode.regmap(mmioReg: _*)
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

class WithBRU(address: BigInt = 0x20000000L, nDomains: Int = 4, withMonitor: Boolean = false, nDramBanks: Int = 8, dramBankOffset: Int = 16) extends Config((_, _, _) => {
  case BRUKey => Some(BRUParams(address = address, nDomains = nDomains, withMonitor = withMonitor, nDramBanks = nDramBanks, dramBankOffset = dramBankOffset))
})
