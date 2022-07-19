/**
 * Rover: blocks
 */
//% color="#0000ff" weight=10 icon="\uf1b9"
//% groups="['LEDs','Motors','Sensors', "IR"]"
namespace mecanumRover {
    const PCA9685_ADDRESS = 0x47
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const ULTRASONIC_TRIG_PIN = DigitalPin.P15
    const ULTRASONIC_ECHO_PIN = DigitalPin.P16
    const ULTRASONIC_SERVO_PIN = AnalogPin.P14
    const LEDS_PIN = DigitalPin.P8
    const IR_PIN = DigitalPin.P9

    const STP_CHA_L = 2047
    const STP_CHA_H = 4095

    const STP_CHB_L = 1
    const STP_CHB_H = 2047

    const STP_CHC_L = 1023
    const STP_CHC_H = 3071

    const STP_CHD_L = 3071
    const STP_CHD_H = 1023

    export enum LineTrackingSensors {
        LEFT = 0,
        RIGHT = 1,
    }

    const LineTrackingSensoPins: { [key: number]: number } = {
        [LineTrackingSensors.LEFT]: DigitalPin.P1,
        [LineTrackingSensors.RIGHT]: DigitalPin.P2,
    }

    export enum Motors {
        FRONT_LEFT = 0,
        FRONT_RIGHT = 1,
        REAR_RIGHT = 2,
        REAR_LEFT = 3
    }

    export enum Leds {
        FRONT_LEFT = 0,
        FRONT_RIGHT = 1,
        REAR_RIGHT = 2,
        REAR_LEFT = 3
    }

    export enum LedGroups {
        ALL = 0,
        FRONT_LEFT = 1,
        FRONT_RIGHT = 2,
        REAR_RIGHT = 3,
        REAR_LEFT = 4,
        FRONT = 5,
        REAR = 6,
        LEFT = 7,
        RIGHT = 8,
    }

    export enum MotorDirections {
        BACKWARD = -1,
        STOP = 0,
        FORWARD = 1,
    }

    export enum MoveDirections {
        FORWARD = 0,
        BACKWARD = 1,
        RIGHT = 2,
        LEFT = 3,
        FORWARD_RIGHT = 4,
        FORWARD_LEFT = 5,
        BACKWARD_RIGHT = 6,
        BACKWARD_LEFT = 7,
        TURN_RIGHT = 8,
        TURN_LEFT = 9,
    }

    //Mapping of led groups to leds
    const LedGroupsMap: { [key: number]: number[] } = {
        [LedGroups.ALL]: [Leds.FRONT_LEFT, Leds.FRONT_RIGHT, Leds.REAR_RIGHT, Leds.REAR_LEFT],
        [LedGroups.FRONT_LEFT]: [Leds.FRONT_LEFT],
        [LedGroups.FRONT_RIGHT]: [Leds.FRONT_RIGHT],
        [LedGroups.REAR_RIGHT]: [Leds.REAR_RIGHT],
        [LedGroups.REAR_LEFT]: [Leds.REAR_LEFT],
        [LedGroups.FRONT]: [Leds.FRONT_LEFT, Leds.FRONT_RIGHT],
        [LedGroups.REAR]: [Leds.REAR_RIGHT, Leds.REAR_LEFT],
        [LedGroups.LEFT]: [Leds.FRONT_LEFT, Leds.REAR_LEFT],
        [LedGroups.RIGHT]: [Leds.FRONT_RIGHT, Leds.REAR_RIGHT],
    }

    export enum LedColors {
        //% block=red
        Red = 0xFF0000,
        //% block=orange
        Orange = 0xFFA500,
        //% block=yellow
        Yellow = 0xFFFF00,
        //% block=green
        Green = 0x00FF00,
        //% block=blue
        Blue = 0x0000FF,
        //% block=indigo
        Indigo = 0x4b0082,
        //% block=violet
        Violet = 0x8a2be2,
        //% block=purple
        Purple = 0xFF00FF,
        //% block=white
        White = 0xFFFFFF,
        //% block=black
        Black = 0x000000
    }


    //Mapping of motor dicrections to PCA9685 addresses
    const motorDirectionsMap: { [key: number]: { [key: number]: number[] } } = {
        [Motors.FRONT_LEFT]: {
            [MotorDirections.FORWARD]: [0x5, 0x3],
            [MotorDirections.STOP]: [0x3, 0x4, 0x5],
            [MotorDirections.BACKWARD]: [0x5, 0x4]
        },
        [Motors.FRONT_RIGHT]: {
            [MotorDirections.FORWARD]: [0x0, 0x1],
            [MotorDirections.STOP]: [0x0, 0x1, 0x2],
            [MotorDirections.BACKWARD]: [0x0, 0x2]
        },
        [Motors.REAR_RIGHT]: {
            [MotorDirections.FORWARD]: [0x6, 0x7],
            [MotorDirections.STOP]: [0x6, 0x7, 0x8],
            [MotorDirections.BACKWARD]: [0x6, 0x8]
        },
        [Motors.REAR_LEFT]: {
            [MotorDirections.FORWARD]: [0xb, 0x9],
            [MotorDirections.STOP]: [0x9, 0xa, 0xb],
            [MotorDirections.BACKWARD]: [0xb, 0xa]
        },
    }

    //Mapping of move direction to motor directions
    const moveDirectionsMap: { [key: number]: { [key: number]: number } } = {
        [MoveDirections.FORWARD]: {
            [Motors.FRONT_LEFT]: MotorDirections.FORWARD,
            [Motors.FRONT_RIGHT]: MotorDirections.FORWARD,
            [Motors.REAR_RIGHT]: MotorDirections.FORWARD,
            [Motors.REAR_LEFT]: MotorDirections.FORWARD
        },
        [MoveDirections.BACKWARD]: {
            [Motors.FRONT_LEFT]: MotorDirections.BACKWARD,
            [Motors.FRONT_RIGHT]: MotorDirections.BACKWARD,
            [Motors.REAR_RIGHT]: MotorDirections.BACKWARD,
            [Motors.REAR_LEFT]: MotorDirections.BACKWARD
        },
        [MoveDirections.RIGHT]: {
            [Motors.FRONT_LEFT]: MotorDirections.FORWARD,
            [Motors.FRONT_RIGHT]: MotorDirections.BACKWARD,
            [Motors.REAR_RIGHT]: MotorDirections.FORWARD,
            [Motors.REAR_LEFT]: MotorDirections.BACKWARD
        },
        [MoveDirections.LEFT]: {
            [Motors.FRONT_LEFT]: MotorDirections.BACKWARD,
            [Motors.FRONT_RIGHT]: MotorDirections.FORWARD,
            [Motors.REAR_RIGHT]: MotorDirections.BACKWARD,
            [Motors.REAR_LEFT]: MotorDirections.FORWARD
        },
        [MoveDirections.FORWARD_RIGHT]: {
            [Motors.FRONT_LEFT]: MotorDirections.FORWARD,
            [Motors.FRONT_RIGHT]: MotorDirections.STOP,
            [Motors.REAR_RIGHT]: MotorDirections.FORWARD,
            [Motors.REAR_LEFT]: MotorDirections.STOP
        },
        [MoveDirections.FORWARD_LEFT]: {
            [Motors.FRONT_LEFT]: MotorDirections.STOP,
            [Motors.FRONT_RIGHT]: MotorDirections.FORWARD,
            [Motors.REAR_RIGHT]: MotorDirections.STOP,
            [Motors.REAR_LEFT]: MotorDirections.FORWARD
        },
        [MoveDirections.BACKWARD_RIGHT]: {
            [Motors.FRONT_LEFT]: MotorDirections.STOP,
            [Motors.FRONT_RIGHT]: MotorDirections.BACKWARD,
            [Motors.REAR_RIGHT]: MotorDirections.STOP,
            [Motors.REAR_LEFT]: MotorDirections.BACKWARD
        },
        [MoveDirections.BACKWARD_LEFT]: {
            [Motors.FRONT_LEFT]: MotorDirections.BACKWARD,
            [Motors.FRONT_RIGHT]: MotorDirections.STOP,
            [Motors.REAR_RIGHT]: MotorDirections.BACKWARD,
            [Motors.REAR_LEFT]: MotorDirections.STOP
        },
        [MoveDirections.TURN_RIGHT]: {
            [Motors.FRONT_LEFT]: MotorDirections.FORWARD,
            [Motors.FRONT_RIGHT]: MotorDirections.BACKWARD,
            [Motors.REAR_RIGHT]: MotorDirections.BACKWARD,
            [Motors.REAR_LEFT]: MotorDirections.FORWARD
        },
        [MoveDirections.TURN_LEFT]: {
            [Motors.FRONT_LEFT]: MotorDirections.BACKWARD,
            [Motors.FRONT_RIGHT]: MotorDirections.FORWARD,
            [Motors.REAR_RIGHT]: MotorDirections.FORWARD,
            [Motors.REAR_LEFT]: MotorDirections.BACKWARD
        },
    }

    let initialized = false
    let lastEchoDuration = 0;

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        //serial.writeValue("ch", channel)
        //serial.writeValue("on", on)
        //serial.writeValue("off", off)

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }

    //% blockId=rover_stop block="Motor stop |%motor=Motors|"
    //% group="Motors"
    //% weight=10
    export function MotorStop(motor: Motors): void {
        MotorRun(motor, MotorDirections.STOP);
    }

    /**
     * Running a motor at a specified speed.
     * @param motor the index of the motor.
     * @param speed the speed of the motor.
     */
    //% blockId=rover_motor_run block="Motor |%motor=Motors| speed %speed"
    //% weight=20
    //% speed.min=-100 speed.max=100
    //% group="Motors"
    export function MotorRun(motor: Motors, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }

        speed = speed * 41; // map 100 to 4096
        speed = Math.constrain(speed, -4095, 4095); //limit the speed range to [-4095; 4095]

        //TODO fix check if motor in motors
        //if (Motors[motor])
        //    return

        let motorDirection
        if (speed > 0) {
            motorDirection = MotorDirections.FORWARD
        } else if (speed < 0) {
            motorDirection = MotorDirections.BACKWARD
        } else {
            motorDirection = MotorDirections.STOP
        }

        for (let motorAddr of motorDirectionsMap[motor][motorDirection]) {
            setPwm(motorAddr, 0, Math.abs(speed))
        }
    }

    //% blockId=rover_move block="Rover move direction |%moveDirection=MoveDirections| speed %speed"
    //% speed.min=0 speed.max=100
    //% speed.defl=20
    //% weight=100
    //% group="Motors"
    export function RoverMove(moveDirection: MoveDirections, speed: number): void {
        //TODO iterate over motor indexes
        for (let motor = 0; motor < Object.keys(moveDirectionsMap[moveDirection]).length; motor++) {
            let motorDirection = moveDirectionsMap[moveDirection][motor]
            MotorRun(motor, speed * motorDirection)
        }
    }

    //% blockId=rover_stop_all block="Rover Stop"
    //% weight=85
    //% blockGap=90
    //% group="Motors"
    export function RoverStop(): void {
        //TODO iterate over motors
        for (let motor = 0; motor < Object.keys(motorDirectionsMap).length; motor++) {
            MotorStop(motor);
        }
    }

    /**
     * Export the distance measured by the ultrasonic module.
     */
    //% blockId=rover_ultrasonic_distance block="distance"
    //% weight=20
    //% group="Sensors"
    export function UltrasonicDistance(): number {
        //send trig pulse
        pins.setPull(ULTRASONIC_TRIG_PIN, PinPullMode.PullNone);
        pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 0)
        control.waitMicros(2);
        pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 1)
        control.waitMicros(10);
        pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 0)

        // read echo pulse  max distance : 6m  
        let t = pins.pulseIn(ULTRASONIC_ECHO_PIN, PulseValue.High, 35000);
        let ret = t;

        if (ret == 0 && lastEchoDuration != 0) {
            ret = lastEchoDuration;
        }
        lastEchoDuration = t;
        return Math.round(ret * 10 / 6 / 58);
    }

    /**
     * Turn the ultrasonic servo.
     */
    //% blockId=rover_ultrasonic_servo_turn block="Ultrasonic servo turn %angle"
    //% angle.min=0 angle.max=100
    //% weight=10
    //% group="Sensors"
    export function UltrasonicServoTurn(angle: number) {
        angle = Math.constrain(angle, 0, 180); //limit the angle range to [0; 180]
        pins.servoWritePin(ULTRASONIC_SERVO_PIN, angle)
    }

    /**
     * Export value of line-tracking sensors.
     * TODO add readme on linetracking value
     */
    //% blockId=rover_line_tracking block="line-racking sensor"
    //% weight=30
    //% group="Sensors"
    export function LineTracking(): number {
        let value = pins.digitalReadPin(LineTrackingSensoPins[LineTrackingSensors.RIGHT]) << 1 | pins.digitalReadPin(LineTrackingSensoPins[LineTrackingSensors.LEFT]) << 0
        return value
    }

    /**
     * Control the bottom leds
     */
    //% blockId=rover_leds_on block="Set leds |%LedGroups| to color |%LedColors| and brightness %brightness"
    //% weight=20
    //% brightness.min=0 brightness.max=255
    //% group="Leds"
    export function LedsOn(leds: LedGroups, color: LedColors, brightness: number) {
        let buffer = pins.createBuffer(12);
        //TODO iterate over LEDs
        buffer.fill(0, 0 * 3, 4 * 3);

        for (let led of LedGroupsMap[leds]){
            const offset = 3 * led

            let red = (color >> 16) & 0xFF;
            let green = (color >> 8) & 0xFF;
            let blue = (color) & 0xFF;

            buffer[offset + 0] = (green * brightness) >> 8;
            buffer[offset + 1] = (red * brightness) >> 8;
            buffer[offset + 2] = (blue * brightness) >> 8;  
        }

        ws2812b.sendBuffer(buffer, LEDS_PIN);
    }

    /**
     * Control the bottom leds
     */
    //% blockId=rover_leds_off block="Switch off the leds"
    //% weight=10
    //% group="Leds"
    export function LedsOff() {
        let buffer = pins.createBuffer(12);
        //TODO iterate over LEDs
        buffer.fill(0, 0 * 3, 4 * 3);
        ws2812b.sendBuffer(buffer, LEDS_PIN);
    }

    /**
     * Get the code of the pressed IR button
     */
    //% blockId=rover_ir_button block="IR button"
    //% weight=20
    //% group="IR"
    export function getIrSignal(): number{
        makerbit.connectIrReceiver(IR_PIN, IrProtocol.Keyestudio)
        return makerbit.irButton()
    }

    /**
     * Execute custom code on IR button pressed
     */
    //% blockId=rover_ir_run block="On IR button |%button| |%action|"
    //% weight=10
    //% group="IR"
    export function runOnIrSignal(button: IrButton, action: IrButtonAction, handler: () => void) {
        makerbit.connectIrReceiver(IR_PIN, IrProtocol.Keyestudio)
        makerbit.onIrButton(button, action, handler)
    }
}