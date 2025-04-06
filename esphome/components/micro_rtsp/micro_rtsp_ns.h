// micro_rtsp_ns.h
#pragma once

#include "esphome/core/component.h"
#include "micro_rtsp.h"

namespace esphome {
namespace micro_rtsp {

struct MicroRTSPCreator {
  MicroRTSP *operator()() { return new MicroRTSP(); }
};

}  // namespace micro_rtsp
}  // namespace esphome
