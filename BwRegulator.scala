package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
//import midas.targetutils.SynthesizePrintf
import org.chipsalliance.cde.config.{Parameters, Field, Config}

import freechips.rocketchip.tile.BRUTileIO

// BRUTileIO defined in BaseTile.scala so we have it everywhere

class BwRegulator()(implicit p: Parameters) extends LazyModule
{
    val device = new SimpleDevice("bru",Seq("bru"))

    // first number is number of cores, second is number of banks..
    // TODO: Can we grab the number of cores from params somehow?
    val ioNode = Seq.fill(4)(BundleBridgeSource(() => new BRUTileIO(p(SubsystemBankedCoherenceKey).nBanks)))
    val dramRegNode = BundleBridgeSink[BRUTileIO](Some(() => Flipped(new BRUTileIO(4))))
    val adapterNode = TLAdapterNode()

    // add simple config registers
    val regnode = new TLRegisterNode(
        address = Seq(AddressSet(0x20000000, 0x7ff)),
        device = device,
        beatBytes = 8)

    lazy val module = new BwRegulatorModule(this)
}

class BwRegulatorModule(outer: BwRegulator) extends LazyModuleImp(outer)
{
  val throttleIO = outer.ioNode.map(_.bundle)

  val nDomains = 4
  val numDramBanks = 8
  val dramBankBitOffset = 16
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

      val isAccessRead = isAcquire || ( countInstrFetch && isInstrFetch )

      doesClientFireAcquire(i) := isAccessRead && in.a.fire && clientRegEnable(i)

      out.a.bits.domainId := clientDomainIds(i)
      out.c.bits.domainId := clientDomainIds(i)

      for ( j <- 0 until numDramBanks ) {
        doesClientAccessBank(i)(j) := ( ( in.a.bits.address >> dramBankBitOffset.U ) & dramBankMask.U ) === j.U
      }

      for ( j <- 0 until numBanks ) {
        throttleIO(i).nThrottle(j) := false.B
      }

      when ( clientRegEnable(i) && globalEnable ) {
          for ( j <- 0 until numDramBanks ) {
              when ( doesClientAccessBank(i)(j) && isAccessRead && outer.dramRegNode.bundle.nThrottle(clientDomainIds(i)) ) {
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