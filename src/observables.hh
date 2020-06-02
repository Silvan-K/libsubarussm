#include <array>
#include <vector>
#include <cstddef>

namespace SSM {

  typedef std::byte Byte;
  typedef std::vector<Byte> Bytes;
  typedef std::array<std::byte, 3> Address;

  // Abstract base class
  
  class Observable {
  public:
    virtual double convert(Bytes::const_iterator input) const = 0;
    virtual std::string unit() const = 0;
    virtual std::vector<Address> addresses() const = 0;
    virtual int numBytes() const = 0;
  };

  typedef std::vector<const SSM::Observable*> Observables;

  // Abstract base class for quantities represented by one byte
  
  class OneByteObservable : public Observable {
  public:
    double convert(Bytes::const_iterator input) const override
    {
      uint8_t raw0 = std::to_integer<uint8_t>(*input);
      return convert(raw0);
    }
    int numBytes() const override { return 1; }
  private:
    virtual double convert(uint8_t) const = 0;
  };

  // Abstract base class for quantities represented two bytes
  
  class TwoByteObservable : public Observable {
  public:
    double convert(Bytes::const_iterator input) const override
    {
      uint8_t raw0 = std::to_integer<uint8_t>(*(input+0));
      uint8_t raw1 = std::to_integer<uint8_t>(*(input+1));
      return convert(raw0 | (raw1 << 8));
    }
    int numBytes() const override { return 2; }
  private:
    virtual double convert(uint16_t) const = 0;
  };

  /////////////////////
  // Derived classes //
  /////////////////////
  
  // Battery Voltage
  // 8 bit value
  // P0x01C = low byte
  // Multiply value by 0.08 to get volts

  class BatteryVoltage : public OneByteObservable {
  public:
    std::string unit() const override { return "Volt"; }
    std::vector<Address> addresses() const override
    { return { Address{std::byte{0x0}, std::byte{0x0}, std::byte{0x1C}}}; }
  private:
    double convert(uint8_t value) const override { return value*0.08; };
  };

  // Manifold Relative Pressure
  // 8 bit value
  // P0x24 = low byte
  // Subtract 128 from value, multiply by 37.0 and divide by 255 to get psig
    
  class ManifoldRelativePressure : public OneByteObservable {
  public:
    std::string unit() const override { return "kPa"; }
    std::vector<Address> addresses() const override
    { return { Address{std::byte{0x0}, std::byte{0x0}, std::byte{0x24}}}; }
  private:
    double convert(uint8_t value) const override { return (value-128.0)*37/255; };
  };

  // Engine Speed
  // 16 bit value
  // P0x0E = high byte
  // P0x0F = low byte
  // Divide value by 4 to get RPM
  
  class EngineSpeed : public TwoByteObservable {
  public:
    std::string unit() const override { return "RPM"; }
    std::vector<Address> addresses() const override
    { return { Address{std::byte{0x0}, std::byte{0x0}, std::byte{0x0F}},
	       Address{std::byte{0x0}, std::byte{0x0}, std::byte{0x0E}}}; }
  private:
    double convert(uint16_t value) const override { return value*0.25; };
  };
  
}
