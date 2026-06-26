#include <atomic>
#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include "pai_onnx_inference/chunk_queue.hpp"

using pai_onnx_inference::ChunkQueue;
using pai_onnx_inference::Action;

TEST(ChunkQueue, PushPopOrder) {
  ChunkQueue q;
  Action a; a.values = {1, 2, 3, 4, 5, 6};
  q.push(a);
  auto b = q.pop_front();
  ASSERT_TRUE(b.has_value());
  EXPECT_EQ(b->values, a.values);
  EXPECT_FALSE(q.pop_front().has_value());
}

TEST(ChunkQueue, ClearEmpties) {
  ChunkQueue q;
  for (int i = 0; i < 3; ++i) {
    Action a; a.values = {double(i), 0, 0, 0, 0, 0}; q.push(a);
  }
  q.clear();
  EXPECT_EQ(q.size(), 0u);
}

TEST(ChunkQueue, ReplacePreviousChunkOnPush) {
  ChunkQueue q;
  for (int i = 0; i < 25; ++i) {
    Action a; a.values = {double(i), 0, 0, 0, 0, 0}; q.push(a);
  }
  Action new_a; new_a.values = {99, 0, 0, 0, 0, 0};
  q.push(new_a);
  EXPECT_EQ(q.size(), 1u);
  auto b = q.pop_front();
  ASSERT_TRUE(b.has_value());
  EXPECT_DOUBLE_EQ(b->values[0], 99.0);
}

TEST(ChunkQueue, ConcurrentPushPopIsSafe) {
  ChunkQueue q;
  constexpr int kN = 1000;
  std::atomic<bool> done{false};
  std::atomic<int> popped{0};
  std::vector<std::thread> writers;
  for (int t = 0; t < 4; ++t) {
    writers.emplace_back([&] {
      for (int i = 0; i < kN; ++i) {
        Action a; a.values = {double(i), 0, 0, 0, 0, 0};
        q.push(a);
      }
    });
  }
  std::thread reader([&] {
    while (!done.load()) {
      if (q.pop_front()) popped.fetch_add(1);
      else std::this_thread::yield();
    }
  });
  for (auto & w : writers) w.join();
  done.store(true);
  reader.join();
  // Since push() replaces the chunk, only actions not-yet-overwritten can be
  // popped. We assert that SOMETHING was popped (no crashes/races) and that
  // the queue is empty at the end.
  EXPECT_GT(popped.load(), 0);
  EXPECT_EQ(q.size(), 0u);
}