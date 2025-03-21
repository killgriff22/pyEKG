import time
import board
MAX30105_INTSTAT1 = 0x00
MAX30105_INTSTAT2 = 0x01
MAX30105_INTENABLE1 = 0x02
MAX30105_INTENABLE2 = 0x03

# FIFO Registers
MAX30105_FIFOWRITEPTR = 0x04
MAX30105_FIFOOVERFLOW = 0x05
MAX30105_FIFOREADPTR = 0x06
MAX30105_FIFODATA = 0x07
# Configuration Registers
MAX30105_FIFOCONFIG = 0x08
MAX30105_MODECONFIG = 0x09
# Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
MAX30105_PARTICLECONFIG = 0x0A
MAX30105_LED1_PULSEAMP = 0x0C
MAX30105_LED2_PULSEAMP = 0x0D
MAX30105_LED3_PULSEAMP = 0x0E
MAX30105_LED_PROX_AMP = 0x10
MAX30105_MULTILEDCONFIG1 = 0x11
MAX30105_MULTILEDCONFIG2 = 0x12

# Die Temperature Registers
MAX30105_DIETEMPINT = 0x1F
MAX30105_DIETEMPFRAC = 0x20
MAX30105_DIETEMPCONFIG = 0x21

# Proximity Function Registers
MAX30105_PROXINTTHRESH = 0x30

# Part ID Registers
MAX30105_REVISIONID = 0xFE
MAX30105_PARTID = 0xFF    # Should always be 0x15. Identical to MAX30102.

# MAX30105 Commands
# Interrupt configuration (pg 13, 14)
MAX30105_INT_A_FULL_MASK = 0b10000000
MAX30105_INT_A_FULL_ENABLE = 0x80
MAX30105_INT_A_FULL_DISABLE = 0x00

MAX30105_INT_DATA_RDY_MASK = 0b01000000
MAX30105_INT_DATA_RDY_ENABLE = 0x40
MAX30105_INT_DATA_RDY_DISABLE = 0x00

MAX30105_INT_ALC_OVF_MASK = 0b00100000
MAX30105_INT_ALC_OVF_ENABLE = 0x20
MAX30105_INT_ALC_OVF_DISABLE = 0x00

MAX30105_INT_PROX_INT_MASK = 0b00010000
MAX30105_INT_PROX_INT_ENABLE = 0x10
MAX30105_INT_PROX_INT_DISABLE = 0x00

MAX30105_INT_DIE_TEMP_RDY_MASK = 0b00000010
MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02
MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00

MAX30105_SAMPLEAVG_MASK = 0b11100000
MAX30105_SAMPLEAVG_1 = 0x00
MAX30105_SAMPLEAVG_2 = 0x20
MAX30105_SAMPLEAVG_4 = 0x40
MAX30105_SAMPLEAVG_8 = 0x60
MAX30105_SAMPLEAVG_16 = 0x80
MAX30105_SAMPLEAVG_32 = 0xA0

MAX30105_ROLLOVER_MASK = 0xEF
MAX30105_ROLLOVER_ENABLE = 0x10
MAX30105_ROLLOVER_DISABLE = 0x00

MAX30105_A_FULL_MASK = 0xF0

# Mode configuration commands (page 19)
MAX30105_SHUTDOWN_MASK = 0x7F
MAX30105_SHUTDOWN = 0x80
MAX30105_WAKEUP = 0x00

MAX30105_RESET_MASK = 0xBF
MAX30105_RESET = 0x40

MAX30105_MODE_MASK = 0xF8
MAX30105_MODE_REDONLY = 0x02
MAX30105_MODE_REDIRONLY = 0x03
MAX30105_MODE_MULTILED = 0x07

# Particle sensing configuration commands (pgs 19-20)
MAX30105_ADCRANGE_MASK = 0x9F
MAX30105_ADCRANGE_2048 = 0x00
MAX30105_ADCRANGE_4096 = 0x20
MAX30105_ADCRANGE_8192 = 0x40
MAX30105_ADCRANGE_16384 = 0x60

MAX30105_SAMPLERATE_MASK = 0xE3
MAX30105_SAMPLERATE_50 = 0x00
MAX30105_SAMPLERATE_100 = 0x04
MAX30105_SAMPLERATE_200 = 0x08
MAX30105_SAMPLERATE_400 = 0x0C
MAX30105_SAMPLERATE_800 = 0x10
MAX30105_SAMPLERATE_1000 = 0x14
MAX30105_SAMPLERATE_1600 = 0x18
MAX30105_SAMPLERATE_3200 = 0x1C

MAX30105_PULSEWIDTH_MASK = 0xFC
MAX30105_PULSEWIDTH_69 = 0x00
MAX30105_PULSEWIDTH_118 = 0x01
MAX30105_PULSEWIDTH_215 = 0x02
MAX30105_PULSEWIDTH_411 = 0x03

# Multi-LED Mode configuration (pg 22)
MAX30105_SLOT1_MASK = 0xF8
MAX30105_SLOT2_MASK = 0x8F
MAX30105_SLOT3_MASK = 0xF8
MAX30105_SLOT4_MASK = 0x8F

SLOT_NONE = 0x00
SLOT_RED_LED = 0x01
SLOT_IR_LED = 0x02
SLOT_GREEN_LED = 0x03
SLOT_NONE_PILOT = 0x04
SLOT_RED_PILOT = 0x05
SLOT_IR_PILOT = 0x06
SLOT_GREEN_PILOT = 0x07

MAX_30105_EXPECTEDPARTID = 0x15


class MAX30105:
    def __init__(self, i2c: board.I2C, address=0x57):
        self._i2c: board.I2C = i2c
        self._i2caddr = address
        self._i2c.try_lock()
        self.setup(0x1F, 4, 2, 100, 411, 4096)
        self.sense = {
            'head': 0,  # Array position to insert new data
            'tail': 0,  # Array position to read next data
            'red': [0] * 32,  # Red and IR LED sensor data
            'IR': [0] * 32,
            'green': [0] * 32,
        }

    def setup(self, powerLevel: int=0x1F,  sampleAverage: int=0,  ledMode: int=0,  sampleRate: int=0,  pulseWidth: int=0,  adcRange: int=0):
        self.softReset()  # Reset all configuration, threshold, and data registers to POR values

        # FIFO Configuration
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        # The chip will average multiple samples of same type together if you wish
        if (sampleAverage == 1):
            self.setFIFOAverage(MAX30105_SAMPLEAVG_1)
        elif (sampleAverage == 2):
            self.setFIFOAverage(MAX30105_SAMPLEAVG_2)
        elif (sampleAverage == 4):
            self.setFIFOAverage(MAX30105_SAMPLEAVG_4)
        elif (sampleAverage == 8):
            self.setFIFOAverage(MAX30105_SAMPLEAVG_8)
        elif (sampleAverage == 16):
            self.setFIFOAverage(MAX30105_SAMPLEAVG_16)
        elif (sampleAverage == 32):
            self.setFIFOAverage(MAX30105_SAMPLEAVG_32)
        else:
            self.setFIFOAverage(MAX30105_SAMPLEAVG_4)

        # setFIFOAlmostFull(2) #Set to 30 samples to trigger an 'Almost Full' interrupt
        self.enableFIFORollover()  # Allow FIFO to wrap/roll over
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        # Mode Configuration
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        if (ledMode == 3):
            self.setLEDMode(MAX30105_MODE_MULTILED)
        elif (ledMode == 2):
            self.setLEDMode(MAX30105_MODE_REDIRONLY)
        else:
            self.setLEDMode(MAX30105_MODE_REDONLY)
        self.activeLEDs = ledMode  # Used to control how many bytes to read from FIFO buffer
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        # Particle Sensing Configuration
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        if (adcRange < 4096):
            self.setADCRange(MAX30105_ADCRANGE_2048)  # 7.81pA per LSB
        elif (adcRange < 8192):
            self.setADCRange(MAX30105_ADCRANGE_4096)  # 15.63pA per LSB
        elif (adcRange < 16384):
            self.setADCRange(MAX30105_ADCRANGE_8192)  # 31.25pA per LSB
        elif (adcRange == 16384):
            self.setADCRange(MAX30105_ADCRANGE_16384)  # 62.5pA per LSB
        else:
            self.setADCRange(MAX30105_ADCRANGE_2048)

        if (sampleRate < 100):
            # Take 50 samples per second
            self.setSampleRate(MAX30105_SAMPLERATE_50)
        elif (sampleRate < 200):
            self.setSampleRate(MAX30105_SAMPLERATE_100)
        elif (sampleRate < 400):
            self.setSampleRate(MAX30105_SAMPLERATE_200)
        elif (sampleRate < 800):
            self.setSampleRate(MAX30105_SAMPLERATE_400)
        elif (sampleRate < 1000):
            self.setSampleRate(MAX30105_SAMPLERATE_800)
        elif (sampleRate < 1600):
            self.setSampleRate(MAX30105_SAMPLERATE_1000)
        elif (sampleRate < 3200):
            self.setSampleRate(MAX30105_SAMPLERATE_1600)
        elif (sampleRate == 3200):
            self.setSampleRate(MAX30105_SAMPLERATE_3200)
        else:
            self.setSampleRate(MAX30105_SAMPLERATE_50)

        # The longer the pulse width the longer range of detection you'll have
        # At 69us and 0.4mA it's about 2 inches
        # At 411us and 0.4mA it's about 6 inches
        if (pulseWidth < 118):
            # Page 26, Gets us 15 bit resolution
            self.setPulseWidth(MAX30105_PULSEWIDTH_69)
        elif (pulseWidth < 215):
            self.setPulseWidth(MAX30105_PULSEWIDTH_118)  # 16 bit resolution
        elif (pulseWidth < 411):
            self.setPulseWidth(MAX30105_PULSEWIDTH_215)  # 17 bit resolution
        elif (pulseWidth == 411):
            self.setPulseWidth(MAX30105_PULSEWIDTH_411)  # 18 bit resolution
        else:
            self.setPulseWidth(MAX30105_PULSEWIDTH_69)
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        # LED Pulse Amplitude Configuration
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        # Default is 0x1F which gets us 6.4mA
        # powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
        # powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
        # powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
        # powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

        self.setPulseAmplitudeRed(powerLevel)
        self.setPulseAmplitudeIR(powerLevel)
        self.setPulseAmplitudeGreen(powerLevel)
        self.setPulseAmplitudeProximity(powerLevel)
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        # Multi-LED Mode Configuration, Enable the reading of the three LEDs
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        self.enableSlot(1, SLOT_RED_LED)
        if (ledMode > 1):
            self.enableSlot(2, SLOT_IR_LED)
        if (ledMode > 2):
            self.enableSlot(3, SLOT_GREEN_LED)
        # enableSlot(1, SLOT_RED_PILOT)
        # enableSlot(2, SLOT_IR_PILOT)
        # enableSlot(3, SLOT_GREEN_PILOT)
        # -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        self.clearFIFO()  # Reset the FIFO before we begin checking the sensor

    def softReset(self):
        self.bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET)
        # Poll for bit to clear, reset is then complete
        # Timeout after 100ms
        startTime = time.time()
        while (time.time() - startTime < 100):
            response = self.readRegister8(self._i2caddr, MAX30105_MODECONFIG)
            if ((response & MAX30105_RESET) == 0):
                break  # We're done!
            time.sleep(1)  # Let's not over burden the I2C bus

    # Given a register, read it, mask it, and then set the thing
    def bitMask(self, reg, mask, thing):
        # Grab current register context
        originalContents = self.readRegister8(self._i2caddr, reg)
        # Zero-out the portions of the register we're interested in
        originalContents = originalContents & mask
        # Change contents
        self.writeRegister8(self._i2caddr, reg, originalContents | thing)

    def readRegister8(self, address, reg):
        buffer = bytearray(1)
        self._i2c.writeto(address, bytes([reg]))
        self._i2c.readfrom_into(address, buffer)
        if (self._i2c.probe(self._i2caddr)):
            self._i2c.readfrom_into(address, buffer)
            return buffer[0]
        return (0)  # Fail

    def writeRegister8(self, address, reg, value):
        self._i2c.writeto(address, bytes([reg, value]))

    def setFIFOAverage(self, numberOfSamples):
        self.bitMask(MAX30105_FIFOCONFIG,
                     MAX30105_SAMPLEAVG_MASK, numberOfSamples)

    def shutDown(self):
        # Put IC into low power mode (datasheet pg. 19)
        # During shutdown the IC will continue to respond to I2C commands but will
        # not update with or take new readings (such as temperature)
        self.bitMask(MAX30105_MODECONFIG,
                     MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN)

    def wakeUp(self):
        # Pull IC out of low power mode (datasheet pg. 19)
        self.bitMask(MAX30105_MODECONFIG,
                     MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP)

    def setLEDMode(self,  mode):
        # Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
        # See datasheet, page 19
        self.bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode)

    def setADCRange(self,  adcRange):
        # adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
        self.bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange)

    def setSampleRate(self,  sampleRate):
        # sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
        self.bitMask(MAX30105_PARTICLECONFIG,
                     MAX30105_SAMPLERATE_MASK, sampleRate)

    def setPulseWidth(self,  pulseWidth):
        # pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
        self.bitMask(MAX30105_PARTICLECONFIG,
                     MAX30105_PULSEWIDTH_MASK, pulseWidth)

    # NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
    # See datasheet, page 21

    def setPulseAmplitudeRed(self,  amplitude):
        self.writeRegister8(self._i2caddr, MAX30105_LED1_PULSEAMP, amplitude)

    def setPulseAmplitudeIR(self,  amplitude):
        self.writeRegister8(self._i2caddr, MAX30105_LED2_PULSEAMP, amplitude)

    def setPulseAmplitudeGreen(self,  amplitude):
        self.writeRegister8(self._i2caddr, MAX30105_LED3_PULSEAMP, amplitude)

    def setPulseAmplitudeProximity(self,  amplitude):
        self.writeRegister8(self._i2caddr, MAX30105_LED_PROX_AMP, amplitude)

    def setProximityThreshold(self,  threshMSB):
        # Set the IR ADC count that will trigger the beginning  of particle-sensing mode.
        # The threshMSB signifies only the 8 most significant-bits of the ADC count.
        # See datasheet, page 24.
        self.writeRegister8(self._i2caddr, MAX30105_PROXINTTHRESH, threshMSB)

    # Given a slot number assign a thing to it
    # Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
    # Assigning a SLOT_RED_LED will pulse LED
    # Assigning a SLOT_RED_PILOT will ??

    def enableSlot(self,  slotNumber, device):
        if slotNumber == 1:
            self.bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device)
        elif slotNumber == 2:
            self.bitMask(MAX30105_MULTILEDCONFIG1,
                         MAX30105_SLOT2_MASK, device << 4)
        elif slotNumber == 3:
            self.bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device)
        elif slotNumber == 4:
            self.bitMask(MAX30105_MULTILEDCONFIG2,
                         MAX30105_SLOT4_MASK, device << 4)

    # Clears all slot assignments

    def disableSlots(self):
        self.writeRegister8(self._i2caddr, MAX30105_MULTILEDCONFIG1, 0)
        self.writeRegister8(self._i2caddr, MAX30105_MULTILEDCONFIG2, 0)

    #
    # FIFO Configuration
    #

    # Set sample average (Table 3, Page 18)

    def setFIFOAverage(self,  numberOfSamples):
        self.bitMask(MAX30105_FIFOCONFIG,
                     MAX30105_SAMPLEAVG_MASK, numberOfSamples)

    # Resets all points to start in a known state
    # Page 15 recommends clearing FIFO before beginning a read

    def clearFIFO(self):
        self.writeRegister8(self._i2caddr, MAX30105_FIFOWRITEPTR, 0)
        self.writeRegister8(self._i2caddr, MAX30105_FIFOOVERFLOW, 0)
        self.writeRegister8(self._i2caddr, MAX30105_FIFOREADPTR, 0)

    # Enable roll over if FIFO over flows

    def enableFIFORollover(self):
        self.bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK,
                     MAX30105_ROLLOVER_ENABLE)

    # Disable roll over if FIFO over flows

    def disableFIFORollover(self):
        self.bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK,
                     MAX30105_ROLLOVER_DISABLE)

    # Set number of samples to trigger the almost full interrupt (Page 18)
    # Power on default is 32 samples
    # Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples

    def setFIFOAlmostFull(self,  numberOfSamples):
        self.bitMask(MAX30105_FIFOCONFIG,
                     MAX30105_A_FULL_MASK, numberOfSamples)

    # Read the FIFO Write Pointer

    def getWritePointer(self):
        return (self.readRegister8(self._i2caddr, MAX30105_FIFOWRITEPTR))

    # Read the FIFO Read Pointer

    def getReadPointer(self):
        return (self.readRegister8(self._i2caddr, MAX30105_FIFOREADPTR))
# Begin Interrupt configuration

    def getINT1(self):
        return (self.readRegister8(self._i2caddr, MAX30105_INTSTAT1))

    def getINT2(self):
        return (self.readRegister8(self._i2caddr, MAX30105_INTSTAT2))

    def enableAFULL(self):
        self.bitMask(MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK,
                     MAX30105_INT_A_FULL_ENABLE)

    def disableAFULL(self):
        self.bitMask(MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK,
                     MAX30105_INT_A_FULL_DISABLE)

    def enableDATARDY(self):
        self.bitMask(MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK,
                     MAX30105_INT_DATA_RDY_ENABLE)

    def disableDATARDY(self):
        self.bitMask(MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK,
                     MAX30105_INT_DATA_RDY_DISABLE)

    def enableALCOVF(self):
        self.bitMask(MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK,
                     MAX30105_INT_ALC_OVF_ENABLE)

    def disableALCOVF(self):
        self.bitMask(MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK,
                     MAX30105_INT_ALC_OVF_DISABLE)

    def enablePROXINT(self):
        self.bitMask(MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK,
                     MAX30105_INT_PROX_INT_ENABLE)

    def disablePROXINT(self):
        self.bitMask(MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK,
                     MAX30105_INT_PROX_INT_DISABLE)

    def enableDIETEMPRDY(self):
        self.bitMask(MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK,
                     MAX30105_INT_DIE_TEMP_RDY_ENABLE)

    def disableDIETEMPRDY(self):
        self.bitMask(MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK,
                     MAX30105_INT_DIE_TEMP_RDY_DISABLE)

    # Polls the sensor for new data
    # Call regularly
    # If new data is available, it updates the head and tail in the main struct
    # Returns number of new samples obtained
    def check(self):
        # Read register FIDO_DATA in (3-byte * number of active LED) chunks
        # Until FIFO_RD_PTR = FIFO_WR_PTR
        I2C_BUFFER_LENGTH = 64
        STORAGE_SIZE = 4  # Each long is 4 bytes so limit this to fit on your micro
        readPointer = self.getReadPointer()
        writePointer = self.getWritePointer()

        numberOfSamples = 0

        # Do we have new data?
        if (readPointer != writePointer):
            # Calculate the number of readings we need to get from sensor
            numberOfSamples = writePointer - readPointer
            if (numberOfSamples < 0):
                numberOfSamples += 32  # Wrap condition

            # We now have the number of readings, now calc bytes to read
            # For this example we are just doing Red and IR (3 bytes each)
            bytesLeftToRead = numberOfSamples * self.activeLEDs * 3

            # Get ready to read a burst of data from the FIFO register
            self._i2c.writeto(self._i2caddr, bytes([MAX30105_FIFODATA]))

            # We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
            # I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
            # Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
            while (bytesLeftToRead > 0):
                toGet = bytesLeftToRead
                if (toGet > I2C_BUFFER_LENGTH):
                    # If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
                    # 32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
                    # 32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

                    # Trim toGet to be a multiple of the samples we need to read
                    toGet = I2C_BUFFER_LENGTH - \
                        (I2C_BUFFER_LENGTH % (self.activeLEDs * 3))

                bytesLeftToRead -= toGet
                arr = bytearray(toGet)
                # Request toGet number of bytes from sensor
                self._i2c.readfrom_into(self._i2caddr, arr)

                while (toGet > 0):
                    # Advance the head of the storage struct
                    self.sense['head'] += 1
                    self.sense['head'] %= STORAGE_SIZE  # Wrap condition

                    # Array of 4 bytes that we will convert into long
                    temp = bytearray(4)
                    tempLong = 0

                    # Burst read three bytes - RED
                    self._i2c.readfrom_into(self._i2caddr, temp)

                    # Convert array to long
                    tempLong = eval(
                        '0x' + ''.join('{:02x}'.format(x) for x in temp))

                    tempLong &= 0x3FFFF  # Zero out all but 18 bits

                    # Store this reading into the sense array
                    self.sense['red'][self.sense['head']] = tempLong

                    if (self.activeLEDs > 1):
                        # Burst read three more bytes - IR
                        self._i2c.readfrom_into(self._i2caddr, temp)

                        # Convert array to long
                        tempLong = eval(
                        '0x' + ''.join('{:02x}'.format(x) for x in temp))

                        tempLong &= 0x3FFFF  # Zero out all but 18 bits

                        self.sense['IR'][self.sense['head']] = tempLong

                    if (self.activeLEDs > 2):
                        # Burst read three more bytes - Green
                        self._i2c.readfrom_into(self._i2caddr, temp)

                        # Convert array to long
                        tempLong = eval(
                        '0x' + ''.join('{:02x}'.format(x) for x in temp))

                        tempLong &= 0x3FFFF  # Zero out all but 18 bits

                        self.sense['green'][self.sense['head']] = tempLong

                    toGet -= self.activeLEDs * 3

        # Let the world know how much new data we found
        return (numberOfSamples)
