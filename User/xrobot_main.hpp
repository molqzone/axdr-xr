#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "MT6701.hpp"

static LibXR::ApplicationManager &XRobotInit(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  static ApplicationManager appmgr;

  // Auto-generated module instantiations
  static MT6701 MT6701_0(hw, appmgr, "mt6701_spi", "mt6701_spi_cs");
  return appmgr;
}

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager &appmgr = XRobotInit(hw);

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(2);
  }
}
