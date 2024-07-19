#include <iostream>
#include "lidro/isl100/frame.hpp"
#include "lidro/isl100/interface.hpp"
#include <sstream>
#include "log_message.h"
#include "lidro/isl100/lutp_protocol.hpp"
#include <unistd.h>
#include <stdexcept>

namespace lidro::isl100 {

Interface::Interface(const lidro::isl100::Config &config) {
  int err;

  deferred_ = std::make_unique<base::SimpleThread>();
  deferred_->Init();
  deferred_->Start();

  stream_mode_ = config.mode;
  ctrl_io_ = std::make_unique<net::udp>();
  ctrl_io_->setAddr(config.host_addr);
  ctrl_io_->setPort(config.host_ctrl_port);
  ctrl_io_->setRemotePort(config.lidar_addr, config.lidar_ctrl_port);
  ctrl_io_->setOnRead([&](uint8_t *buf, int len) {
      LOG(INFO) << "response";
      H((char *)buf, len);
  });

  err = ctrl_io_->init();
  if (err) {
    throw std::runtime_error("exception ctrl-io");
  }
  ctrl_io_->start();

  data_io_ = std::make_unique<net::udp>();
  data_io_->setAddr(config.host_addr);
  data_io_->setPort(config.host_data_port);
  data_io_->setOnRead(
    std::bind(
      &Interface::onRead,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
  
  err = data_io_->init();
  if (err) {
    throw std::runtime_error("exception data-io");
  }
  data_io_->start();

  restartStream();
}

Interface::~Interface() {
  stopStream();
}

int Interface::restartStream() {
  int err;
  err = stopStream();
  err = startStream();
  return err;
}

int Interface::startStream() {
  LUTP header{};
  header.version = 0xFFEE;
  header.prod_id = 0x41A2;
  stream_mode mode = stream_mode_;
  if (mode == stream_mode_1) {
    header.strm_type = 0x0180;
    header.opaque = 0x0900;
  }
  else if (mode == stream_mode_2) {
    header.strm_type = 0x0180;
    header.opaque = 0x0a00;
  }
  else if (mode == stream_mode_3) {
    header.strm_type = 0x0180;
    header.opaque = 0x0b00;
  }
  else if (mode == stream_mode_4) {
    header.strm_type = 0x0180;
    header.opaque = 0x0c00;
  }
  else {
    throw std::runtime_error("unhandled stream-mode");
  }

  size_t n = ctrl_io_->send((uint8_t *)&header, sizeof(header));
  if(n != sizeof(header)) {
    LOG(ERROR) << "[REQ] failed to send the start command";
    return -1;
  }
  LOG(INFO) << "[REQ] start";
  return 0;
}

int Interface::stopStream() {
  LUTP header{};
  header.version = 0xFFEE;
  header.prod_id = 0x41A2;
  header.opaque = 0x0400;
  header.strm_type = 0x0180;
  size_t n = ctrl_io_->send((uint8_t *)&header, sizeof(header));
  if(n != sizeof(header)) {
    LOG(ERROR) << "[REQ] failed to send the start command";
    return -1;
  }
  LOG(INFO) << "[REQ] stop";
  return 0;
}

void Interface::onRead(uint8_t *buf, int len) {
  LUTP *head = (LUTP *)buf;
  uint8_t *payload = buf + sizeof(LUTP);
  if(head->frag_no != (current_frag_no_)) {
    LOG(WARNING) << "not sync. current frag. no:" << head->frag_no << 
    ", expected frag. no: " << current_frag_no_;
  }

  if(head->frag_no < (head->frag_total)) {
    if(!frame_) {
      frame_ = std::make_unique<Frame>(stream_mode_);
      current_frag_no_ = 0;
      sumof_frag_ = 0;
      expected_sumof_frag_ = (head->frag_total-1)*head->frag_total / 2;
    }

    current_frag_no_ = head->frag_no;
    int n = frame_->fetch(head->frag_no, payload, (size_t)head->len);
    assert(n == head->len);
    current_frag_no_++;
    current_frag_no_ %= head->frag_total;
    sumof_frag_ += head->frag_no;
    if(head->frag_no == (head->frag_total-1)) {
      if(on_frame_ && frame_&& (sumof_frag_ == expected_sumof_frag_)) {
        deferred_->PostTask([&]() {
          int err = on_frame_(std::move(frame_));
          if(err) {
            LOG(WARNING) << "error while processing payload";
          }
        });
      }
      else {
        frame_ = nullptr;
        LOG(WARNING) << "sumof_frag: " << sumof_frag_ << ", expected_sumof_frag_: " << expected_sumof_frag_;
      }
    }
  }
}

} // end namespace lidro
