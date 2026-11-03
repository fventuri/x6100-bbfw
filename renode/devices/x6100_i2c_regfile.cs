//
// x6100-bbfw -- byte-addressed register-file I2C slave for Renode bring-up.
//
// Models the firmware's observed I2C transaction (i2c.md): the master writes a
// one-byte register pointer, issues a repeated START, then reads N bytes with
// auto-increment. An address-only probe (write of zero payload bytes) simply
// ACKs because the device is attached at its address.
//
// The optional "Profile" property preloads recognizable, repeatable bytes so a
// read returns something meaningful (e.g. MAX17205 identity/status). Values are
// placed at the byte offsets the firmware extracts from its 0x00-block read.
//
// Load with:  include @renode/devices/x6100_i2c_regfile.cs
// Attach in a repl:  max17205: I2C.X6100RegFileI2C @ i2c1 0x36 { Profile: "max17205" }
//
using System;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class X6100RegFileI2C : II2CPeripheral
    {
        public X6100RegFileI2C()
        {
            data = new byte[0x200];
            Reset();
        }

        public void Reset()
        {
            pointer = 0;
        }

        public void Write(byte[] packet)
        {
            if(packet == null || packet.Length == 0)
            {
                return; // address-only probe: just ACK
            }
            pointer = packet[0];
            for(int i = 1; i < packet.Length; i++)
            {
                data[(pointer + i - 1) & 0x1FF] = packet[i];
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(int i = 0; i < count; i++)
            {
                result[i] = data[(pointer + i) & 0x1FF];
            }
            pointer = (pointer + count) & 0x1FF;
            return result;
        }

        public void FinishTransmission()
        {
        }

        public string Profile
        {
            get { return profile; }
            set
            {
                profile = value;
                if(value == "max17205")
                {
                    // Byte offsets within the 0x00 register block (i2c.md):
                    data[0x00] = 0x02; data[0x01] = 0x00;   // Status (POR-ish, nonzero)
                    data[0x0C] = 0x80; data[0x0D] = 0x47;   // RepSOC  0x4780 (~71 %)
                    data[0x10] = 0x60; data[0x11] = 0x16;   // Temp    0x1660 (~22 C)
                    data[0x16] = 0xEC; data[0x17] = 0xFF;   // AvgCurrent 0xFFEC (signed -20)
                    data[0x42] = 0x05; data[0x43] = 0x41;   // DevName (register 0x21)
                    data[0x1B4] = 0x00; data[0x1B5] = 0x19; // Batt 0x1900=6400 (x1.25 = 8000 mV)
                }
            }
        }

        private readonly byte[] data;
        private int pointer;
        private string profile = "";
    }
}
