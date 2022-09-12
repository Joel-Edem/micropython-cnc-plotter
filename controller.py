import time
from machine import SPI, Pin, PWM
import uasyncio


class Point:

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z


class ControllerConfig:
    PWM_PIN = 19
    SPI_ID = 2
    SPI_DATA = 23
    SPI_CLK = 18
    SPI_LATCH = 5
    SR_NUM_REGISTERS = 2
    SR_CLEAR = None
    SR_ENABLE = None

    X_MAX_STEP = 255
    Y_MAX_STEP = 255
    Z_MAX_STEP = 255
    Z_MIN_STEP = 220
    X_SPEED = 100
    Y_SPEED = 100
    Z_SPEED = 100
    StepsPerMillimeter = 5.2

    TOOL_UP_POSITION = 255
    TOOL_DOWN_POSITION = 120

    @classmethod
    def steps_to_mm(cls, steps):
        return steps / cls.StepsPerMillimeter

    @classmethod
    def mm_to_steps(cls, length):
        return length * cls.StepsPerMillimeter


class ShiftRegister:

    def __init__(self, ):
        """
        num_registers:
        ser: Data Pin
        rclk: Latch/Storage Register Clock
        srclk: Shift Register Clock.
        oe: Output Enable pin (Active LOW). Set High to disable
        srclr: Shift Register Clear. Set LOW to clear register
        """
        self.spi = SPI(ControllerConfig.SPI_ID, 100000,
                       bits=8 * ControllerConfig.SR_NUM_REGISTERS,
                       sck=Pin(ControllerConfig.SPI_CLK, ),
                       mosi=Pin(ControllerConfig.SPI_DATA, ),
                       miso=None)
        self.buf = bytearray(ControllerConfig.SR_NUM_REGISTERS)
        self.rclk = Pin(ControllerConfig.SPI_LATCH, Pin.OUT)

        self.srclr = Pin(ControllerConfig.SR_CLEAR, Pin.OUT, value=1) if ControllerConfig.SR_CLEAR else None
        self.oe = Pin(ControllerConfig.SR_ENABLE, Pin.OUT, value=0) if ControllerConfig.SR_ENABLE else None

    def _write(self, latch=False):
        self.spi.write(self.buf)
        if latch:
            self.latch()

    def latch(self):
        self.rclk(1)
        self.rclk(0)

    def get_pin(self, pin):
        return (self.buf[pin // 8] >> (pin % 8)) & 1

    def set_pin(self, pin, value=None, latch=True):
        if value:
            self.buf[pin // 8] |= (1 << (pin % 8))
        else:
            self.buf[pin // 8] &= ~(1 << (pin % 8))
        self._write(latch)

    def set_pins(self, pins, values):
        self.set_pin(pins[0], values[0], False)
        self.set_pin(pins[1], values[2], False)
        self.set_pin(pins[2], values[1], False)
        self.set_pin(pins[3], values[3], False)
        self.latch()

    def clear(self):
        if self.srclr:
            self.srclr(0)
            self.srclr(1)
        else:
            for i in range(ControllerConfig.SR_NUM_REGISTERS * 8):
                self.set_pin(i, 0, False)
        self.latch()

    def enable(self, enabled=True):
        if self.oe is None:
            raise RuntimeError('oe pin is required')
        self.oe(not enabled)


class LN293D:

    def __init__(self, en1a, en1b, en2a, en2b, ):
        self.en1a = Pin(en1a, Pin.OUT, value=0)
        self.en1b = Pin(en1b, Pin.OUT, value=0)
        self.en2a = Pin(en2a, Pin.OUT, value=0)
        self.en2b = Pin(en2b, Pin.OUT, value=0)
        self.pins = (self.en1a, self.en1b, self.en2a, self.en2b)

    async def set_pins(self, pins, vals, wait=None):
        while wait and (time.ticks_diff(time.ticks_us(), wait) < 0):
            await uasyncio.sleep(0)
        for pin, val in zip(pins, vals):
            self.pins[pin].value(val)


class DCMotor:

    def __init__(self, en1, en2, driver, reverse=False):
        self.pins = (en1, en2)
        self.driver = driver
        self._fwd = (1, 0)
        self._off_state = (0, 0)
        self.state = None
        self.off()

        if reverse:
            self.reverse()

    def switch_direction(self):
        self._fwd = tuple([1 ^ i for i in self._fwd])

    def reverse(self):
        self.state = tuple([1 ^ i for i in self._fwd])
        self.run()

    def forward(self):
        self.state = self._fwd
        self.run()

    def run(self):
        self.driver.set_pin(self.pins[0], self.state[0])
        self.driver.set_pin(self.pins[1], self.state[1])

    def off(self):
        self.state = self._off_state
        self.run()


# noinspection DuplicatedCode
class StepperMotor:
    PATTERN = ((1, 1, 0, 0),
               (0, 1, 1, 0),
               (0, 0, 1, 1),
               (1, 0, 0, 1))

    def __init__(self, en1a, en1b, en2a, en2b, driver,
                 speed=100, max_steps=255, min_steps=0, reverse=False, ):
        """
        :param en1a:
        :param en1b:
        :param en2a:
        :param en2b:
        :param speed: speed in steps per second
        :param max_steps:
        :param min_steps:
        :param driver:
        """
        self.pins = (en1a, en1b, en2a, en2b,)
        self.driver = driver

        self.max_steps = max_steps
        self.min_steps = min_steps
        self.speed = speed

        self.cur_idx = 0
        self.pattern = self.PATTERN
        self.ptn_len = len(self.pattern) - 1
        if reverse:
            self.reverse()

        self.cur_steps = self.min_steps
        self.last_tick = time.ticks_us()
        self.is_off = True
        uasyncio.run(self.off())  # turn off

    async def off(self):
        await self.set_pins(values=(0, 0, 0, 0))
        self.is_off = True

    async def on(self):
        await self.set_pins(self.pattern[self.cur_idx])
        self.is_off = False

    def reverse(self):
        """
        Reverse motor direction
        :return:
        """
        pattern = list(self.pattern)
        pattern.reverse()
        self.pattern = tuple(pattern)

    async def set_pins(self, values, wait=None):
        while wait and (time.ticks_diff(time.ticks_us(), wait) < 0):
            await uasyncio.sleep(0)
        self.driver.set_pins(self.pins, values)
        self.last_tick = time.ticks_us()

    def zero(self):
        self.cur_steps = 0

    async def step_to(self, pos, speed=None):
        # check current pos
        # if curr pos is ahead back
        # if curr is behind mv fwd
        if pos > self.max_steps:
            pos = self.max_steps
        if pos < self.min_steps:
            pos = self.min_steps
        num_steps = pos - self.cur_steps
        await self.step(num_steps, speed)
        return self.cur_steps

    async def step(self, steps, sps=None, ):

        steps = min(max(steps, self.min_steps - self.cur_steps), self.max_steps - self.cur_steps)

        mode = self.pattern
        ptn_len = self.ptn_len
        idx = self.cur_idx
        sc = 0  # step count
        step_time = int(round(1000000 / max(1, (sps or self.speed)), 0))

        if time.ticks_diff(time.ticks_us(), time.ticks_add(self.last_tick, step_time)) >= 0:
            wait_for = time.ticks_us()
        else:
            wait_for = time.ticks_add(self.last_tick, step_time)
        await self.on()

        if steps < 0:
            for s in range(abs(steps)):
                if idx == 0:
                    idx = ptn_len
                else:
                    idx -= 1
                await self.set_pins(mode[idx], wait_for)
                wait_for = time.ticks_add(wait_for, step_time)
                sc -= 1
        else:
            for s in range(steps):
                if idx == ptn_len:
                    idx = 0
                else:
                    idx += 1
                await self.set_pins(mode[idx], wait_for)
                wait_for = time.ticks_add(wait_for, step_time)
                sc += 1
        await self.off()
        self.cur_idx = idx
        self.cur_steps += sc
        return self.cur_steps


class CNC:
    HIGH_SPEED = 1.3

    def __init__(self, home=False):
        self.driver = ShiftRegister()
        self.driver.clear()
        self.pwm_pin = PWM(Pin(ControllerConfig.PWM_PIN), freq=512)
        self.pwm_pin.duty_u16(65535)
        self.parser = GCodeParser()

        self.motors = {
            "x": StepperMotor(8, 9, 10, 11, self.driver,
                              max_steps=ControllerConfig.X_MAX_STEP,
                              speed=ControllerConfig.X_SPEED, ),
            "y": StepperMotor(0, 1, 2, 3, self.driver,
                              max_steps=ControllerConfig.Y_MAX_STEP,
                              speed=ControllerConfig.Y_SPEED, ),
            "z": StepperMotor(4, 5, 6, 7, self.driver,
                              max_steps=ControllerConfig.Z_MAX_STEP,
                              min_steps=ControllerConfig.Z_MIN_STEP,
                              speed=ControllerConfig.Z_SPEED),
        }
        self.tool = DCMotor(12, 13, self.driver)

        self.position = Point(
            x=self.motors['x'].cur_steps,
            y=self.motors['y'].cur_steps,
            z=self.motors['y'].cur_steps
        )
        self.drawing_pos = Point()

        if home:
            uasyncio.run(self.home())

    async def off(self, axis=None):
        self.pwm_pin.duty_u16(0)

        if not axis:
            await uasyncio.gather(*(m.off() for m in self.motors.values()))
        else:
            await self.motors[axis].off()

    async def on(self, axis=None):
        self.pwm_pin.duty_u16(65535)

        if not axis:
            await uasyncio.gather(*(m.on() for m in self.motors.values()))
        else:
            await self.motors[axis].on()

    async def step_to(self, axis, position, speed=None):
        """

        :param axis: int|tuple
        :param position: int|tuple
        :param speed:
        :return:
        """
        if isinstance(axis, str):
            return await self.motors[axis].step_to(position, speed)
        t = []

        if isinstance(position, int):
            if speed is None or isinstance(speed, int):
                for a in axis:
                    t.append(self.motors[a].step_to(position, speed))
            else:
                for idx, a in enumerate(axis):
                    t.append(self.motors[a].step_to(position, speed[idx]))
        else:
            if speed is None or isinstance(speed, int):
                for idx, a in enumerate(axis):
                    t.append(self.motors[a].step_to(position[idx], speed))
            else:
                for idx, a in enumerate(axis):
                    t.append(self.motors[a].step_to(position[idx], speed[idx]))
        return await uasyncio.gather(*t)

    async def home(self):
        tasks = (m.step(m.max_steps, ) for m in self.motors.values())
        await uasyncio.gather(*tasks)
        await uasyncio.sleep(1)
        tasks = (m.step(-m.max_steps, ) for m in self.motors.values())
        await uasyncio.gather(*tasks)
        [m.zero() for m in self.motors.values()]

    def run(self, fn):
        return uasyncio.run(self._run(fn))

    async def _run(self, fn):
        async for cmd in self.parser.parse_file(fn):
            self.handle_cmds(cmd)

    def handle_cmds(self, parts):
        print(f"Processing  parts==> {parts}")
        if "U" in parts:
            self.tool_up()
        elif "D" in parts:
            self.tool_down()
        elif "G" in parts:
            self.handle_g_parts(parts)
        elif "M" in parts:
            self.handle_m_parts(parts)
        else:
            print(f"UNHANDLED G-CODE {parts}")

    def tool_up(self):
        uasyncio.run(self.motors["y"].step_to(ControllerConfig.TOOL_UP_POSITION))
        self.set_draw_pos('z', self.position.z)

    def tool_down(self):
        uasyncio.run(self.motors["y"].step_to(ControllerConfig.TOOL_DOWN_POSITION))
        self.set_draw_pos('z', self.position.z)

    def handle_g_parts(self, parts):
        if parts["G"] == 0 or parts["G"] == 1:  # set fast speed
            if len([1 for p in parts if p.upper() in ("X", "Y", "Z")]) > 0:
                point = Point(
                    x=parts.get("X", self.drawing_pos.x),
                    y=parts.get("Y", self.drawing_pos.y),
                    z=parts.get("Z", self.drawing_pos.z)
                )
                speed = 0 if parts["G"] == 0 else None
                uasyncio.run(self.draw_line(point, speed))
            else:
                print(f"UNHANDLED G-CODE {parts}")
        elif parts["G"] == 2 or parts["G"] == 3:  # DRAW Circle
            point = ()
            direction = "ccw" if parts["G"] == 3 else "cw"
            self.draw_circle(point, parts["I"], direction)
        elif parts["G"] == 4:  # DWELL
            secs = parts.get("P", parts.get("X", parts.get("U", 0)) * 1000)
            uasyncio.run(self.dwell(secs))
        elif parts["G"] == 28:  # HOME
            if len(parts['values']):  # should be x, y or z
                key = f"{parts['values'][0]}_axis".lower()
                if key:
                    self.home_axis(key)
        else:
            print(f"UNHANDLED G-CODE {parts}")

    def draw_circle(self, point, rad, direction='cw', speed=None):
        raise NotImplementedError

    def home_axis(self, axis):
        uasyncio.run(self.motors[axis].step_to(self.motors[axis].min_steps))
        self.set_draw_pos(axis, self.motors[axis].cur_steps)

    async def draw_line(self, point, speed=None):
        if speed == 0:
            x_speed = self.motors["x"].speed * self.HIGH_SPEED
            y_speed = self.motors["y"].speed * self.HIGH_SPEED
            z_speed = self.motors["z"].speed * self.HIGH_SPEED
        else:
            x_speed, y_speed, z_speed = speed
        x_steps = ControllerConfig.mm_to_steps(point.x)
        y_steps = ControllerConfig.mm_to_steps(point.y)
        z_steps = ControllerConfig.mm_to_steps(point.z)
        tasks = [
            self.motors["y"].step_to(z_steps, y_speed),
            self.motors["x"].step_to(x_steps, x_speed),
            self.motors["y"].step_to(y_steps, z_speed),

        ]
        await uasyncio.gather(*tasks)
        self.drawing_pos.x = point.x
        self.drawing_pos.y = point.y
        self.drawing_pos.z = point.z

    @staticmethod
    async def dwell(duration_ms):
        await uasyncio.sleep(duration_ms)

    def handle_m_parts(self, parts):
        print("HANDLING M CODE")
        try:
            if parts['M'] == 300:
                if parts['S'] == 30:
                    self.tool_down()
                elif parts['S'] == 50:
                    self.tool_up()
            elif parts['M'] == 114:
                print(
                    f"Absolute position : {self.position}"
                )
            else:
                print(f"UNHANDLED M-CODE {parts}")
        except IndexError:
            print(f"UNHANDLED M-CODE {parts}")

    def set_draw_pos(self, axis, steps):
        """
        !!! does not move the motors
        :param axis:
        :param steps: in steps
        :return:
        """
        val = ControllerConfig.steps_to_mm(steps)
        setattr(self.drawing_pos, axis, val)


# noinspection DuplicatedCode
class GCodeParser:

    async def parse_file(self, fn):

        with open(fn, 'r') as f:
            try:
                for line in f:
                    if line:
                        yield self.parse_line(line)
            except EOFError:
                print(f"could not open file {fn}")
                return

    @staticmethod
    def parse_line(line: str):
        clean_line = line.split("(")[0].strip().split(';')[0].strip()
        vals = clean_line.split(" ")
        parts = {"values": []}
        for p in vals:
            if len(p) < 2:
                parts['values'].append(p.upper())
            else:
                try:
                    parts[p[0].upper()] = (int(p[1:]) if "." not in p[1:] else float(p[1:]))
                except ValueError:
                    parts[p[0].upper()] = p[1:].upper()
        return parts


def test():

    cnc = CNC(home=True)
    time.sleep(1)
    cnc.run("test.gcode")

    uasyncio.run(cnc.off())


test()
