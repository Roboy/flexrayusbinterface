#include "flexrayusbinterface/Muscle.hpp"

#include <gtest/gtest.h>

TEST(Slot, batch)
{
  std::mutex m;
  Slot<int>::data_t data;

  Slot<int> slot{ m, data };

  auto g = slot.enqueue(4);
  EXPECT_EQ(g.wait_for(std::chrono::seconds{ 0 }), std::future_status::timeout);

  auto f = slot.enqueue(3);

  EXPECT_EQ(f.wait_for(std::chrono::seconds{ 0 }), std::future_status::timeout);

  EXPECT_EQ(g.wait_for(std::chrono::seconds{ 0 }), std::future_status::ready);
  EXPECT_EQ(g.get(), Completion::Preempted);

  {
    std::lock_guard<std::mutex> _{ m };
    EXPECT_TRUE(static_cast<bool>(data));
    EXPECT_EQ(data->second, 3);
    data->first.set_value(Completion::Completed);
  }

  EXPECT_EQ(f.wait_for(std::chrono::seconds{ 0 }), std::future_status::ready);
  EXPECT_EQ(f.get(), Completion::Completed);
}

TEST(Slot, parallel)
{
  std::mutex m;
  Slot<std::string>::data_t message;

  Slot<std::string> slot{ m, message };

  auto g = slot.enqueue("Hello world!");
}

TEST(Entangled, batch)
{
  std::packaged_task<int(int, int)> sum{ [](int a, int b) { return a + b; } };
  Entangled<int, int> dmc{ 3, sum.get_future() };

  std::packaged_task<int(int, int)> bum{ [](int a, int b) { return a + b; } };
  Entangled<int, int, std::true_type> blah{ 3, bum.get_future() };

  EXPECT_EQ(dmc.status(), std::future_status::timeout);

  sum(2, 4);
  EXPECT_EQ(dmc.status(), std::future_status::ready);
  auto res = std::move(dmc).get();
  EXPECT_EQ(res.first, 3);
  EXPECT_EQ(res.second, 6);
}
