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
    (msg_head_.header.seq - 1 == msg_tail_.header.seq);
}

void TwistMsgBuffer::insert(geometry_msgs::TwistStamped& msg)
{
  geometry_msgs::TwistStamped& tmp;
  /* Swap the head and the tail */
  &tmp       = &msg_head_;
  &msg_head_ = &msg_tail_;
  &msg_tail_ = &tmp;
  
  /* Copy the message to the head */
  &msg_head_ = &msg;
  
  if (msgs_until_initialized_ != 0) {
    --msgs_until_initialized_;
  }
}

void TwistMsgBuffer::flush()
{
  msgs_until_initialized_ = 2;
}

};