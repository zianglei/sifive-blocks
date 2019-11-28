// See LICENSE for license details.
package sifive.blocks.devices.gpio

import Chisel._
import sifive.blocks.devices.pinctrl.{PinCtrl, Pin, BasePin, EnhancedPin, EnhancedPinCtrl}
import sifive.blocks.util.BasicBusBlocker
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{AsyncResetRegVec, SynchronizerShiftReg, SimpleRegIO}

// This is sort of weird because
// the IOF end up at the RocketChipTop
// level, and we have to do the pinmux
// outside of RocketChipTop.
// It would be better if the IOF were here and
// we could do the pinmux inside.

class GPIOPortIO(val c: GPIOParams) extends Bundle {
  val pins = Vec(c.width, new EnhancedPin())
  val iof_0 = if (c.includeIOF) Some(Vec(c.width, new IOFPin).flip) else None
  val iof_1 = if (c.includeIOF) Some(Vec(c.width, new IOFPin).flip) else None
}

case class GPIOParams(
  address: BigInt,
  width: Int,
  includeIOF: Boolean = false)

/** The base GPIO peripheral functionality, which uses the regmap API to
  * abstract over the bus protocol to which it is being connected
  */
abstract class GPIO(busWidthBytes: Int, c: GPIOParams)(implicit p: Parameters)
    extends IORegisterRouter(
      RegisterRouterParams(
        name = "gpio",
        compat = Seq("sifive,gpio0", "sifive,gpio1"),
        base = c.address,
        beatBytes = busWidthBytes),
      new GPIOPortIO(c))
    with HasInterruptSources {

  def nInterrupts = c.width
  override def extraResources(resources: ResourceBindings) = Map(
    "gpio-controller"      -> Nil,
    "#gpio-cells"          -> Seq(ResourceInt(2)),
    "interrupt-controller" -> Nil,
    "#interrupt-cells"     -> Seq(ResourceInt(2)))

  lazy val module = new LazyModuleImp(this) {

  //--------------------------------------------------
  // CSR Declarations
  // -------------------------------------------------

  // SW Control only.
  val portReg = Reg(init = UInt(0, c.width))

  val oeReg  = Module(new AsyncResetRegVec(c.width, 0))
  oeReg.io.en := false.B
  oeReg.io.d := 0.U
  val pueReg = Module(new AsyncResetRegVec(c.width, 0))
  pueReg.io.en := false.B
  pueReg.io.d := 0.U
  val dsReg  = Reg(init = UInt(0, c.width))
  val ieReg  = Module(new AsyncResetRegVec(c.width, 0))
  ieReg.io.en := false.B
  ieReg.io.d := 0.U

  // Synchronize Input to get valueReg
  val inVal = Wire(UInt(0, width=c.width))
  inVal := Vec(port.pins.map(_.i.ival)).asUInt
  val inSyncReg  = SynchronizerShiftReg(inVal, 3, Some("inSyncReg"))
  val valueReg   = Reg(init = UInt(0, c.width), next = inSyncReg)

  // Interrupt Configuration
  val highIeReg = Reg(init = UInt(0, c.width))
  val lowIeReg  = Reg(init = UInt(0, c.width))
  val riseIeReg = Reg(init = UInt(0, c.width))
  val fallIeReg = Reg(init = UInt(0, c.width))
  val highIpReg = Reg(init = UInt(0, c.width))
  val lowIpReg  = Reg(init = UInt(0, c.width))
  val riseIpReg = Reg(init = UInt(0, c.width))
  val fallIpReg = Reg(init = UInt(0, c.width))
  val passthruHighIeReg = Reg(init = UInt(0, c.width))
  val passthruLowIeReg  = Reg(init = UInt(0, c.width))

  // HW IO Function
  val iofEnReg  = Module(new AsyncResetRegVec(c.width, 0))
  iofEnReg.io.en := false.B
  iofEnReg.io.d := 0.U
  val iofSelReg = Reg(init = UInt(0, c.width))
  
  // Invert Output
  val xorReg    = Reg(init = UInt(0, c.width))

  //--------------------------------------------------
  // CSR Access Logic (most of this section is boilerplate)
  // -------------------------------------------------

  val rise = ~valueReg & inSyncReg;
  val fall = valueReg & ~inSyncReg;


  
  def rwReg32base(total: Int, bb: SimpleRegIO, desc: Option[RegFieldDesc] = None) : RegField =
  {  RegField((if (total>32) 32 else total), bb.q, RegWriteFn((valid, data) => {
      when (valid) { 
        bb.d := Cat(bb.q(total-1, 32), data) 
        bb.en := true.B
      }
      Bool(true)
    }), desc)}
  def rwReg32plus(total: Int, bb: SimpleRegIO, desc: Option[RegFieldDesc] = None) : RegField =
  {  RegField((total-32), (bb.q >> 32), RegWriteFn((valid, data) => {
      when (valid) { 
        bb.d := Cat(data, bb.q(31,0))
        bb.en := true.B
      } 
      Bool(true)
    }), desc)}
  // Note that these are out of order.
  def reg32plus(total: Int, regg: UInt, desc: RegFieldDesc) : RegField =
  {
    val maxx = if (total > 32) 31 else (total-1)
    RegField((total-32), regg(total-1, 32), RegWriteFn { (valid, data) => {
      val reggbools = regg.asBools.take(32)
      val databools = data.asBools
      when (valid) {regg := Vec(reggbools ++ databools).asUInt}
      Bool(true)
    }}, desc)
  }
  def reg32base(total: Int, regg: UInt, desc: RegFieldDesc) : RegField =
  {
    val maxx = if (total > 32) 31 else (total-1)
    RegField((maxx+1), regg(maxx, 0), RegWriteFn { (valid, data) => {
      val reggbools = regg.asBools.drop(maxx+1)
      val databools = data.asBools
      when (valid) {regg := Vec(databools ++ reggbools).asUInt}
      Bool(true)
    }}, desc)
  }
  val iofEnFields =  if (c.includeIOF) (Seq(rwReg32base(c.width, iofEnReg.io,
                        Some(RegFieldDesc("iof_en","HW I/O functon enable", reset=Some(0))))))
                     else (Seq(RegField(c.width)))
  val iofSelFields = if (c.includeIOF) (Seq(reg32base(c.width, iofSelReg,
                        RegFieldDesc("iof_sel","HW I/O function select", reset=Some(0)))))
                     else (Seq(RegField(c.width)))
  val iofEnFields1 =  if (c.includeIOF) (Seq(rwReg32plus(c.width, iofEnReg.io,
                        Some(RegFieldDesc("iof_en","HW I/O functon enable", reset=Some(0))))))
                     else (Seq(RegField(c.width)))
  val iofSelFields1 = if (c.includeIOF) (Seq(reg32plus(c.width, iofSelReg,
                        RegFieldDesc("iof_sel","HW I/O function select", reset=Some(0)))))
                     else (Seq(RegField(c.width)))
  def w1ToClearbase(total: Int, reg: UInt, set: UInt, desc: Option[RegFieldDesc] = None): RegField =
    RegField((if (total>32) 32 else total), reg, RegWriteFn((valid, data) => { reg := ~(~reg | Mux(valid, data, UInt(0))) | set; Bool(true) }),
      desc.map{_.copy(access = RegFieldAccessType.RW, wrType=Some(RegFieldWrType.ONE_TO_CLEAR), volatile = true)})
  def w1ToClearplus(total: Int, reg: UInt, set: UInt, desc: Option[RegFieldDesc] = None): RegField =
    RegField((total-32), reg, RegWriteFn((valid, data) => { reg := ~(~reg | Mux(valid, (data<<32), UInt(0))) | set; Bool(true) }),
      desc.map{_.copy(access = RegFieldAccessType.RW, wrType=Some(RegFieldWrType.ONE_TO_CLEAR), volatile = true)})
  val width1 = if (c.width > 32) (c.width - 32) else 0
  val width0 = if (c.width > 32) 32 else c.width
  regmap(
    GPIOCtrlRegs.value     -> Seq(RegField.r(width0, valueReg,
                                  RegFieldDesc("input_value","Pin value", volatile=true))),
    (GPIOCtrlRegs.value + 4) -> Seq(RegField.r(width1, valueReg >> 32,
                                  RegFieldDesc("input_value","Pin value", volatile=true))),
    GPIOCtrlRegs.output_en -> Seq(rwReg32base(c.width, oeReg.io,
                                  Some(RegFieldDesc("output_en","Pin output enable", reset=Some(0))))),
    (GPIOCtrlRegs.output_en+4)-> Seq(rwReg32plus(c.width, oeReg.io,
                                  Some(RegFieldDesc("output_en","Pin output enable", reset=Some(0))))),
    GPIOCtrlRegs.rise_ie   -> Seq(reg32base(c.width, riseIeReg,
                                  RegFieldDesc("rise_ie","Rise interrupt enable", reset=Some(0)))),
    (GPIOCtrlRegs.rise_ie + 4)  -> Seq(reg32plus(c.width, riseIeReg,
                                  RegFieldDesc("rise_ie","Rise interrupt enable", reset=Some(0)))),
    GPIOCtrlRegs.rise_ip   -> Seq(w1ToClearbase(c.width, riseIpReg, rise,
                                  Some(RegFieldDesc("rise_ip","Rise interrupt pending", volatile=true)))),
    (GPIOCtrlRegs.rise_ip+4)   -> Seq(w1ToClearplus(c.width, riseIpReg, rise,
                                  Some(RegFieldDesc("rise_ip","Rise interrupt pending", volatile=true)))),
    GPIOCtrlRegs.fall_ie   -> Seq(reg32base(c.width, fallIeReg,
                                  RegFieldDesc("fall_ie", "Fall interrupt enable", reset=Some(0)))),
    (GPIOCtrlRegs.fall_ie + 4)  -> Seq(reg32plus(c.width, fallIeReg,
                                  RegFieldDesc("fall_ie", "Fall interrupt enable", reset=Some(0)))),
    GPIOCtrlRegs.fall_ip   -> Seq(w1ToClearbase(c.width, fallIpReg, fall,
                                  Some(RegFieldDesc("fall_ip","Fall interrupt pending", volatile=true)))),
    (GPIOCtrlRegs.fall_ip+4)   -> Seq(w1ToClearplus(c.width, fallIpReg, fall,
                                  Some(RegFieldDesc("fall_ip","Fall interrupt pending", volatile=true)))),
    GPIOCtrlRegs.high_ie   -> Seq(reg32base(c.width, highIeReg,
                                  RegFieldDesc("high_ie","High interrupt enable", reset=Some(0)))),
    (GPIOCtrlRegs.high_ie + 4)  -> Seq(reg32plus(c.width, highIeReg,
                                  RegFieldDesc("high_ie","High interrupt enable", reset=Some(0)))),
    GPIOCtrlRegs.high_ip   -> Seq(w1ToClearbase(c.width, highIpReg, valueReg,
                                  Some(RegFieldDesc("high_ip","High interrupt pending", volatile=true)))),
    (GPIOCtrlRegs.high_ip +4)  -> Seq(w1ToClearplus(c.width, highIpReg, valueReg,
                                  Some(RegFieldDesc("high_ip","High interrupt pending", volatile=true)))),
    GPIOCtrlRegs.low_ie    -> Seq(reg32base(c.width, lowIeReg,
                                  RegFieldDesc("low_ie","Low interrupt enable", reset=Some(0)))),
    (GPIOCtrlRegs.low_ie + 4)   -> Seq(reg32plus(c.width, lowIeReg,
                                  RegFieldDesc("low_ie","Low interrupt enable", reset=Some(0)))),
    GPIOCtrlRegs.low_ip    -> Seq(w1ToClearbase(c.width,lowIpReg, ~valueReg,
                                  Some(RegFieldDesc("low_ip","Low interrupt pending", volatile=true)))),
    (GPIOCtrlRegs.low_ip+4)    -> Seq(w1ToClearplus(c.width,lowIpReg, ~valueReg,
                                  Some(RegFieldDesc("low_ip","Low interrupt pending", volatile=true)))),
    GPIOCtrlRegs.port      -> Seq(reg32base(c.width, portReg,
                                  RegFieldDesc("output_value","Output value", reset=Some(0)))),
    (GPIOCtrlRegs.port  + 4)    -> Seq(reg32plus(c.width, portReg,
                                  RegFieldDesc("output_value","Output value", reset=Some(0)))),
    GPIOCtrlRegs.pullup_en -> Seq(rwReg32base(c.width, pueReg.io,
                                  Some(RegFieldDesc("pue","Internal pull-up enable", reset=Some(0))))),
    (GPIOCtrlRegs.pullup_en + 4) -> Seq(rwReg32plus(c.width, pueReg.io,
                                  Some(RegFieldDesc("pue","Internal pull-up enable", reset=Some(0))))),
    GPIOCtrlRegs.iof_en    -> iofEnFields,
    GPIOCtrlRegs.iof_sel   -> iofSelFields,
    (GPIOCtrlRegs.iof_en+4)    -> iofEnFields1,
    (GPIOCtrlRegs.iof_sel+4)   -> iofSelFields1,
    GPIOCtrlRegs.drive     -> Seq(reg32base(c.width, dsReg,
                                  RegFieldDesc("ds","Pin drive strength selection", reset=Some(0)))),
    (GPIOCtrlRegs.drive  + 4)   -> Seq(reg32plus(c.width, dsReg,
                                  RegFieldDesc("ds","Pin drive strength selection", reset=Some(0)))),
    GPIOCtrlRegs.input_en  -> Seq(rwReg32base(c.width, ieReg.io,
                                  Some(RegFieldDesc("input_en","Pin input enable", reset=Some(0))))),
    (GPIOCtrlRegs.input_en + 4) -> Seq(rwReg32plus(c.width, ieReg.io,
                                  Some(RegFieldDesc("input_en","Pin input enable", reset=Some(0))))),
    GPIOCtrlRegs.out_xor   -> Seq(reg32base(c.width, xorReg,
                                  RegFieldDesc("out_xor","Output XOR (invert) enable", reset=Some(0)))),
    (GPIOCtrlRegs.out_xor + 4)  -> Seq(reg32plus(c.width, xorReg,
                                  RegFieldDesc("out_xor","Output XOR (invert) enable", reset=Some(0)))),
    GPIOCtrlRegs.passthru_high_ie -> Seq(reg32base(c.width, passthruHighIeReg,
                                         RegFieldDesc("passthru_high_ie", "Pass-through active-high interrupt enable", reset=Some(0)))),
    (GPIOCtrlRegs.passthru_high_ie + 4) -> Seq(reg32plus(c.width, passthruHighIeReg,
                                         RegFieldDesc("passthru_high_ie", "Pass-through active-high interrupt enable", reset=Some(0)))),
    GPIOCtrlRegs.passthru_low_ie  -> Seq(reg32base(c.width, passthruLowIeReg,
                                         RegFieldDesc("passthru_low_ie", "Pass-through active-low interrupt enable", reset=Some(0)))),
    (GPIOCtrlRegs.passthru_low_ie + 4) -> Seq(reg32plus(c.width, passthruLowIeReg,
                                         RegFieldDesc("passthru_low_ie", "Pass-through active-low interrupt enable", reset=Some(0))))
  )

  //--------------------------------------------------
  // Actual Pinmux
  // -------------------------------------------------

  val swPinCtrl = Wire(Vec(c.width, new EnhancedPinCtrl()))

  // This strips off the valid.
  val iof0Ctrl = Wire(Vec(c.width, new IOFCtrl()))
  val iof1Ctrl = Wire(Vec(c.width, new IOFCtrl()))

  val iofCtrl = Wire(Vec(c.width, new IOFCtrl()))
  val iofPlusSwPinCtrl = Wire(Vec(c.width, new EnhancedPinCtrl()))

  for (pin <- 0 until c.width) {

    // Software Pin Control
    swPinCtrl(pin).pue    := pueReg.io.q(pin)
    swPinCtrl(pin).oval   := portReg(pin)
    swPinCtrl(pin).oe     := oeReg.io.q(pin)
    swPinCtrl(pin).ds     := dsReg(pin)
    swPinCtrl(pin).ie     := ieReg.io.q(pin)

    val pre_xor = Wire(new EnhancedPinCtrl())

    if (c.includeIOF) {
      // Allow SW Override for invalid inputs.
      iof0Ctrl(pin)      <> swPinCtrl(pin)
      when (port.iof_0.get(pin).o.valid) {
        iof0Ctrl(pin)    <> port.iof_0.get(pin).o
      }

      iof1Ctrl(pin)      <> swPinCtrl(pin)
      when (port.iof_1.get(pin).o.valid) {
        iof1Ctrl(pin)    <> port.iof_1.get(pin).o
      }

      // Select IOF 0 vs. IOF 1.
      iofCtrl(pin)       <> Mux(iofSelReg(pin), iof1Ctrl(pin), iof0Ctrl(pin))

      // Allow SW Override for things IOF doesn't control.
      iofPlusSwPinCtrl(pin) <> swPinCtrl(pin)
      iofPlusSwPinCtrl(pin) <> iofCtrl(pin)
   
      // Final XOR & Pin Control
      pre_xor  := Mux(iofEnReg.io.q(pin), iofPlusSwPinCtrl(pin), swPinCtrl(pin))
    } else {
      pre_xor := swPinCtrl(pin)
    }

    port.pins(pin).o      := pre_xor
    port.pins(pin).o.oval := pre_xor.oval ^ xorReg(pin)

    // Generate Interrupts
    interrupts(pin) := (riseIpReg(pin) & riseIeReg(pin)) |
                         (fallIpReg(pin) & fallIeReg(pin)) |
                         (highIpReg(pin) & highIeReg(pin)) |
                         (lowIpReg(pin) & lowIeReg(pin)) |
                         (valueReg(pin) & passthruHighIeReg(pin)) |
                         (~valueReg(pin) & passthruLowIeReg(pin))

    if (c.includeIOF) {
      // Send Value to all consumers
      port.iof_0.get(pin).i.ival := inSyncReg(pin)
      port.iof_1.get(pin).i.ival := inSyncReg(pin)
    }
  }}
}

class TLGPIO(busWidthBytes: Int, params: GPIOParams)(implicit p: Parameters)
  extends GPIO(busWidthBytes, params) with HasTLControlRegMap

case class GPIOAttachParams(
  gpio: GPIOParams,
  controlBus: TLBusWrapper,
  intNode: IntInwardNode,
  blockerAddr: Option[BigInt] = None,
  controlXType: ClockCrossingType = NoCrossing,
  intXType: ClockCrossingType = NoCrossing,
  mclock: Option[ModuleValue[Clock]] = None,
  mreset: Option[ModuleValue[Bool]] = None)
  (implicit val p: Parameters)

object GPIO {
  val nextId = { var i = -1; () => { i += 1; i} }

  def attach(params: GPIOAttachParams): TLGPIO = {
    implicit val p = params.p
    val name = s"gpio_${nextId()}"
    val cbus = params.controlBus
    val gpio = LazyModule(new TLGPIO(cbus.beatBytes, params.gpio))
    gpio.suggestName(name)

    cbus.coupleTo(s"device_named_$name") { bus =>
      val blockerNode = params.blockerAddr.map(BasicBusBlocker(_, cbus, cbus.beatBytes, name))
      (gpio.controlXing(params.controlXType)
        := TLFragmenter(cbus)
        := blockerNode.map { _ := bus } .getOrElse { bus })
    }
    params.intNode := gpio.intXing(params.intXType)
    InModuleBody { gpio.module.clock := params.mclock.map(_.getWrappedValue).getOrElse(cbus.module.clock) }
    InModuleBody { gpio.module.reset := params.mreset.map(_.getWrappedValue).getOrElse(cbus.module.reset) }

    gpio
  }

  def attachAndMakePort(params: GPIOAttachParams): ModuleValue[GPIOPortIO] = {
    val gpio = attach(params)
    val gpioNode = gpio.ioNode.makeSink()(params.p)
    InModuleBody { gpioNode.makeIO()(ValName(gpio.name)) }
  }

  def loopback(g: GPIOPortIO)(pinA: Int, pinB: Int) {
    g.pins.foreach {p =>
      p.i.ival := Mux(p.o.oe, p.o.oval, p.o.pue) & p.o.ie
    }
    val a = g.pins(pinA)
    val b = g.pins(pinB)
    // This logic is not QUITE right, it doesn't handle all the subtle cases.
    // It is probably simpler to just hook a pad up here and use attach()
    // to model this properly.
    a.i.ival := Mux(b.o.oe, (b.o.oval | b.o.pue), (a.o.pue | (a.o.oe & a.o.oval))) & a.o.ie
    b.i.ival := Mux(a.o.oe, (a.o.oval | b.o.pue), (b.o.pue | (b.o.oe & b.o.oval))) & b.o.ie
  }
}
