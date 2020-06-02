#include <array>
#include <vector>

namespace SSM {

  typedef std::array<std::byte, 3> Address;

  // Abstract base classes
  
  class Observable {
  public:
    virtual double convert(unsigned char* it) const = 0;
    virtual std::string unit() const = 0;
    virtual std::vector<Address> addresses() const = 0;
  };

  typedef std::vector<const SSM::Observable*> Observables;

  // Quantities represented by one byte
  
  class OneByteObservable : public Observable {
  public:
    double convert(unsigned char* it) const override
    {
      uint8_t raw0(*it); ++it;
      return convert(raw0);
    }
  private:
    virtual double convert(uint8_t) const = 0;
  };

  // Quantities represented by one byte
  
  class TwoByteObservable : public Observable {
  public:
    double convert(unsigned char* it) const override
    {
      uint8_t raw0(*it); ++it;
      uint8_t raw1(*it); ++it;
      return convert(raw0 | (raw1 << 8));
    }
  private:
    virtual double convert(uint16_t) const = 0;
  };

  // Derived classes

  class BatteryVoltage : public OneByteObservable {
  public:
    std::string unit() const override { return "Volt"; }
    std::vector<Address> addresses() const override
    { return { Address{std::byte{0x0}, std::byte{0x0}, std::byte{0x1C}}}; }
  private:
    double convert(uint8_t value) const override { return value*0.08; };
  };

  class EngineSpeed : public TwoByteObservable {
  public:
    std::string unit() const override { return "Volt"; }
    std::vector<Address> addresses() const override
    { return { Address{std::byte{0x0}, std::byte{0x0}, std::byte{0x0E}},
	       Address{std::byte{0x0}, std::byte{0x0}, std::byte{0x0F}}}; }
  private:
    double convert(uint16_t value) const override { return value*0.25; };
  };
  
}
