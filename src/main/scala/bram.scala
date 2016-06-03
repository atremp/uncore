package uncore

import Chisel._
import cde.{Parameters, Field}
import junctions._
import HastiConstants._

class BRAMSlave(depth: Int)(implicit val p: Parameters) extends Module
  with HasTileLinkParameters {
  val io = new ClientUncachedTileLinkIO().flip

  val bram = SeqMem(depth, Bits(width = tlDataBits))

  val (s0_get :: s0_getblk :: s0_put :: s0_putblk :: Nil) = Seq(
      Acquire.getType, Acquire.getBlockType, Acquire.putType, Acquire.putBlockType
    ).map(io.acquire.bits.isBuiltInType _)

  val fire_acq = io.acquire.fire()
  val fire_gnt = io.grant.fire()

  val multibeat = Reg(init = Bool(false))
  when (fire_acq) {
    multibeat := s0_getblk
  }

  val s0_valid = io.acquire.valid || multibeat
  val s1_valid = Reg(next = s0_valid, init = Bool(false))
  val s1_acq = RegEnable(io.acquire.bits, fire_acq)

  val s0_addr = Cat(io.acquire.bits.addr_block, io.acquire.bits.addr_beat)
  val s1_beat = s1_acq.addr_beat + Mux(io.grant.ready, UInt(1), UInt(0))
  val s1_addr = Cat(s1_acq.addr_block, s1_beat)
  val raddr = Mux(multibeat, s1_addr, s0_addr)

  val last = (s1_acq.addr_beat === UInt(tlDataBeats-1))
  val ren = (io.acquire.valid && (s0_get || s0_getblk)) || (multibeat && !last)
  val wen = (io.acquire.valid && (s0_put || s0_putblk))

  val rdata = bram.read(raddr, ren)
  val wdata = io.acquire.bits.data
  val wmask = io.acquire.bits.wmask()
  assert(!wen || (wmask === Fill(tlDataBytes, Bool(true))),
    "bram: subblock writes not supported")
  when (wen) {
    bram.write(s0_addr, wdata)
  }

  when (multibeat && fire_gnt) {
    s1_acq.addr_beat := s1_beat
    when (last) {
      multibeat := Bool(false)
    }
  }

  io.grant.valid := s1_valid
  io.grant.bits := Grant(
    is_builtin_type = Bool(true),
    g_type = s1_acq.getBuiltInGrantType(),
    client_xact_id = s1_acq.client_xact_id,
    manager_xact_id = UInt(0),
    addr_beat = s1_acq.addr_beat,
    data = rdata)

  val stall = multibeat || (io.grant.valid && !io.grant.ready)
  io.acquire.ready := !stall
}

class NastiRAM(depth: Int)(implicit p: Parameters) extends NastiModule()(p) {
  val io = (new NastiIO).flip

  val nastiDataBytes = nastiXDataBits / 8

  val ram = Mem(depth, Vec(nastiDataBytes, Bits(width = 8)))

  val max_size = log2Ceil(nastiDataBytes)
  val wmask_lut = Vec.tabulate(max_size + 1) { sz => UInt((1 << (1 << sz)) - 1) }
  val addr = Reg(UInt(width = nastiXAddrBits))
  val size = Reg(UInt(width = nastiXSizeBits))
  val len = Reg(UInt(width = nastiXLenBits))
  val wmask = (wmask_lut(size) << addr(max_size - 1, 0)) & io.w.bits.strb

  val wdata = Vec.tabulate(nastiDataBytes)(i => io.w.bits.data(8*(i+1)-1,8*i))
  val rdata = ram(addr >> UInt(max_size)).toBits

  val id = Reg(UInt(width = nastiXIdBits))

  val (s_idle :: s_read :: s_write_data :: s_write_resp :: Nil) =
    Enum(Bits(), 4)
  val state = Reg(init = s_idle)

  io.aw.ready := (state === s_idle)
  io.ar.ready := (state === s_idle) && !io.aw.valid
  io.w.ready := (state === s_write_data)

  io.b.valid := (state === s_write_resp)
  io.b.bits := NastiWriteResponseChannel(id = id)

  io.r.valid := (state === s_read)
  io.r.bits := NastiReadDataChannel(
    id = id,
    data = rdata,
    last = (len === UInt(0)))

  when (io.aw.fire()) {
    addr := io.aw.bits.addr
    size := io.aw.bits.size
    len := io.aw.bits.len
    id := io.aw.bits.id
    state := s_write_data
  }

  when (io.w.fire()) {
    val real_wmask = Vec(wmask.toBools.slice(0, nastiDataBytes))
    ram.write(addr >> UInt(max_size), wdata, real_wmask)
    addr := addr + (UInt(1) << size)
    len := len - UInt(1)
    when (len === UInt(0)) { state := s_write_resp }
  }

  when (io.b.fire()) { state := s_idle }

  when (io.ar.fire()) {
    addr := io.ar.bits.addr
    size := io.ar.bits.size
    len := io.ar.bits.len
    id := io.ar.bits.id
    state := s_read
  }

  when (io.r.fire()) {
    addr := addr + (UInt(1) << size)
    len := len - UInt(1)
    when (len === UInt(0)) { state := s_idle }
  }
}

class HastiRAM(depth: Int)(implicit p: Parameters) extends HastiModule()(p) {
  val io = new HastiSlaveIO

  val wdata = Vec.tabulate(hastiDataBytes)(i => io.hwdata(8*(i+1)-1,8*i))
  val waddr = Reg(UInt(width = hastiAddrBits))
  val wvalid = Reg(init = Bool(false))
  val wsize = Reg(UInt(width = SZ_HSIZE))
  val ram = SeqMem(depth, Vec(hastiDataBytes, Bits(width = 8)))

  val max_size = log2Ceil(hastiDataBytes)
  val wmask_lut = MuxLookup(wsize, SInt(-1, hastiDataBytes).asUInt,
    (0 until max_size).map(sz => (UInt(sz) -> UInt((1 << (1 << sz)) - 1))))
  val wmask = (wmask_lut << waddr(max_size - 1, 0))(hastiDataBytes - 1, 0)

  val is_trans = io.hsel && (io.htrans === HTRANS_NONSEQ || io.htrans === HTRANS_SEQ)
  val raddr = io.haddr >> UInt(max_size)
  val ren = is_trans && !io.hwrite
  val bypass = Reg(init = Bool(false))

  when (is_trans && io.hwrite) {
    waddr := io.haddr
    wsize := io.hsize
    wvalid := Bool(true)
  } .otherwise { wvalid := Bool(false) }

  when (ren) { bypass := wvalid && (waddr >> UInt(max_size)) === raddr }

  when (wvalid) {
    ram.write(waddr >> UInt(max_size), wdata, wmask.toBools)
  }

  val rdata = ram.read(raddr, ren)
  io.hrdata := Cat(rdata.zip(wmask.toBools).zip(wdata).map {
    case ((rbyte, wsel), wbyte) => Mux(wsel && bypass, wbyte, rbyte)
  }.reverse)

  io.hready := Bool(true)
  io.hresp := HRESP_OKAY
}
