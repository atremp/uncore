package uncore

import Chisel._
import junctions._
import cde.{Parameters, Field}

case class DelayPair(min: Int, max: Int)

case object DRAMSimDelay extends Field[DelayPair]
case object DRAMSize extends Field[Long]
case object DRAMBanks extends Field[Int]

class DRAMBank(depth: Int)(implicit p: Parameters) extends Module {
  val io = (new NastiIO).flip

  val delayMin = p(DRAMSimDelay).min
  val delayMax = p(DRAMSimDelay).max
  val delayRange = delayMax - delayMin

  require(isPow2(delayRange), "DRAMBank delay range must be power of 2")

  val next_delay = UInt(delayMin) + LFSR16()(log2Up(delayRange) - 1, 0)
  val delay = Reg(init = UInt(0, 16))
  val inflight = Reg(init = Bool(false))
  val responding = Reg(init = Bool(false))

  val ram = Module(new NastiRAM(depth))
  ram.io.ar.valid := io.ar.valid && !inflight
  ram.io.ar.bits := io.ar.bits
  io.ar.ready := ram.io.ar.ready && !inflight
  ram.io.aw.valid := io.aw.valid && !inflight
  ram.io.aw.bits := io.aw.bits
  io.aw.ready := ram.io.aw.ready && !inflight

  ram.io.w <> io.w

  io.r.valid := ram.io.r.valid && responding
  io.r.bits := ram.io.r.bits
  ram.io.r.ready := io.r.ready && responding

  io.b.valid := ram.io.b.valid && responding
  io.b.bits := ram.io.b.bits
  ram.io.b.ready := io.b.ready && responding

  when (io.ar.fire() || io.aw.fire()) {
    inflight := Bool(true)
    delay := next_delay
  }

  when (inflight && !responding) {
    delay := delay - UInt(1)
    when (delay === UInt(0)) { responding := Bool(true) }
  }

  when (io.r.fire() && io.r.bits.last || io.b.fire()) {
    responding := Bool(false)
    inflight := Bool(false)
  }
}


class DRAMSim(implicit p: Parameters) extends NastiModule()(p) {
  val io = (new NastiIO).flip

  val nBanks = p(DRAMBanks)
  val totalBytes = p(DRAMSize)
  val bankBytes = totalBytes / nBanks
  val bankDepth = (bankBytes / (nastiXDataBits / 8)).toInt

  val banks = Seq.fill(nBanks) { Module(new DRAMBank(bankDepth)) }

  def addrToBank(addr: UInt): UInt = {
    val offset = p(CacheBlockOffsetBits)
    val bankId = addr(log2Up(nBanks) + offset - 1, offset)
    UIntToOH(bankId)
  }

  def cut_addr(addr: UInt): UInt = {
    val offset = p(CacheBlockOffsetBits)
    val low_addr = addr(offset - 1, 0)
    val high_addr = addr >> (log2Up(nBanks) + offset)
    Cat(high_addr, low_addr)
  }

  val router = Module(new NastiRouter(nBanks, addrToBank))
  router.io.master <> io
  banks.zip(router.io.slave).map { case (bank, slave) =>
    bank.io.ar.valid := slave.ar.valid
    bank.io.ar.bits := slave.ar.bits
    bank.io.ar.bits.addr := cut_addr(slave.ar.bits.addr)
    slave.ar.ready := bank.io.ar.ready

    bank.io.aw.valid := slave.aw.valid
    bank.io.aw.bits := slave.aw.bits
    bank.io.aw.bits.addr := cut_addr(slave.aw.bits.addr)
    slave.aw.ready := bank.io.aw.ready

    bank.io.w <> slave.w
    slave.r <> bank.io.r
    slave.b <> bank.io.b
  }

  /* Make sure burst transfers don't cross block boundaries.
   * Otherwise, if there are multiple banks, the later bursts will
   * be coming from the wrong bank */
  def req_ok(ac: NastiAddressChannel): Bool = {
    val offset = p(CacheBlockOffsetBits)
    val total_bytes = (UInt(1) << ac.size) * ac.len

    val start_block = ac.addr >> UInt(offset)
    val end_block = (ac.addr + total_bytes) >> UInt(offset)

    start_block === end_block
  }

  assert(!io.ar.valid || req_ok(io.ar.bits), "Invalid DRAM read")
  assert(!io.aw.valid || req_ok(io.aw.bits), "Invalid DRAM write")
}
