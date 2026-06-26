#include <gtest/gtest.h>
#include "pai_onnx_inference/aligned_buffer.hpp"

using pai_onnx_inference::AlignedBuffer;
using pai_onnx_inference::AlignedFrame;
using pai_onnx_inference::Payload;

TEST(AlignedBuffer, HoldsLastValueOnPop) {
  AlignedBuffer buf(/*tol_ms=*/100);
  buf.push("a", /*stamp_ns=*/1000, Payload{int{1}});
  buf.push("a", /*stamp_ns=*/2000, Payload{int{2}});

  auto f = buf.pop_at(/*now_ns=*/2500);
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->get<int>("a"), 2);
}

TEST(AlignedBuffer, ReturnsNulloptWhenNoData) {
  AlignedBuffer buf(100);
  EXPECT_FALSE(buf.pop_at(1000).has_value());
}

TEST(AlignedBuffer, DiscardsStaleValues) {
  AlignedBuffer buf(/*tol_ms=*/100);
  buf.push("a", /*stamp_ns=*/1000, Payload{int{1}});
  auto f = buf.pop_at(/*now_ns=*/1'001'000'000);
  EXPECT_FALSE(f.has_value());
}

TEST(AlignedBuffer, MultiKeyIndependence) {
  AlignedBuffer buf(100);
  buf.push("a", 1000, Payload{int{1}});
  buf.push("b", 1500, Payload{int{9}});
  auto f = buf.pop_at(2000);
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->get<int>("a"), 1);
  EXPECT_EQ(f->get<int>("b"), 9);
}

TEST(AlignedBuffer, MissingKeyThrowsOnGet) {
  AlignedBuffer buf(100);
  buf.push("a", 1000, Payload{int{1}});
  auto f = buf.pop_at(1500);
  ASSERT_TRUE(f.has_value());
  EXPECT_THROW(f->get<int>("missing"), std::out_of_range);
}