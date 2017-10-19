#include <ras_group8_odometry/TwistMsgBuffer.hpp>

namespace ras_group8_odometry
{

TwistMsgBuffer::TwistMsgBuffer()
    : msg_head_(&msg_a_),
      msg_tail_(&msg_b_)
{
  flush();
}

/* Destructor.
 */
TwistMsgBuffer::~TwistMsgBuffer()
{
  
}

bool TwistMsgBuffer::isInitialized()
{
  return msgs_until_initialized_ == 0;
}

bool TwistMsgBuffer::isValid()
{
  return isInitialized() &&
    (msg_head_->header.seq - 1 == msg_tail_->header.seq);
}

void TwistMsgBuffer::insert(const geometry_msgs::TwistStamped& msg)
{
  const geometry_msgs::TwistStamped* tmp_head = msg_head_;
  /* Swap the head and the tail */
  msg_head_ = msg_tail_;
  msg_tail_ = tmp_head;
  
  /* Copy the message to the head */
  std::memcpy((void *) msg_head_, &msg, sizeof(geometry_msgs::TwistStamped));
  
  if (msgs_until_initialized_ != 0) {
    --msgs_until_initialized_;
  }
}

void TwistMsgBuffer::flush()
{
  msgs_until_initialized_ = 2;
}

}