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

class BwRegulator()(implicit p: Parameters) extends LazyModule
{
    val device = new SimpleDevice("bru",Seq("bru"))

    // first number is number of cores, second is number of banks
    // TODO: Can we grab the number of cores from params somehow?
    val ioNode = Seq.fill(4)(BundleBridgeSource(() => new BRUTileIO(p(SubsystemBankedCoherenceKey).nBanks)))
    //val dramRegNode = BundleBridgeSink[BRUPerBankTileIO](Some(() => Flipped(new BRUPerBankTileIO(4, 8))))
    val coreAccessNode = Seq.fill(4)(BundleBridgeSink[BRUTileAccessIO](Some(() => Flipped(new BRUTileAccessIO(p(SubsystemBankedCoherenceKey).nBanks)))))
    val adapterNode = TLAdapterNode()

    lazy val module = new BwRegulatorModule(this)
}

class BwRegulatorModule(outer: BwRegulator) extends LazyModuleImp(outer)
{
  val throttleIO = outer.ioNode.map(_.bundle)
  
  val nDomains = 4
  val numDramBanks = 8
  val numCacheBanks = 2
  val dramBankBitOffset = 16
  val dramBankMask = numDramBanks - 1

  val memBase = p(ExtMem).get.master.base.U

  val adapterNode = outer.adapterNode
  val nClients = adapterNode.in.length
  println(s"Number of edges into BRU: $nClients")


  for ( i <- 0 until nClients ) {
      val (out, edge_out) = adapterNode.out(i)
      val (in, edge_in) = adapterNode.in(i)

      out <> in

      out.a.bits.domainId := i.U
      out.c.bits.domainId := i.U
        for (j <- 0 until numCacheBanks ) {
          throttleIO(i).nThrottle(j) := false.B
        }

    }
}

trait CanHavePeripheryBRU {
    val BwRegulator: Option[BwRegulator]
}

trait CanHaveBRU { this: BaseSubsystem =>
    private val pbus = locateTLBusWrapper(PBUS)
    private val sbus = locateTLBusWrapper(SBUS)

    private val portName = "bru-mmio"
}
