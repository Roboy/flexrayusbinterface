#include "flexrayusbinterface/Message.hpp"
#include <gtest/gtest.h>

TEST(Message, add)
{
  Message<25> m;
  const char a = 'a';
  const short b = 'b';
  const int c = 'c';
  const double d = 'd';
  static_assert(m.size == 0, "m is not empty");
  auto ma = m.add(a);
  static_assert(ma.size == sizeof(a), "ma is not empty");
  auto mab = ma.add(b);
  static_assert(mab.size == sizeof(b) + ma.size, "mab is not right");
  auto mabc = mab.add(c);
  static_assert(mabc.size == sizeof(c) + mab.size, "mabc is not right");
  auto mabcd = mabc.add(d);
  static_assert(mabcd.size == sizeof(d) + mabc.size, "mabcd is not right");

  char _a;
  short _b;
  int _c;
  double _d;

  std::stringstream ss;
  mabcd.write(ss);

  Parser<25>{}.add(_a).add(_b).add(_c).add(_d).read(ss);

  EXPECT_EQ(a, _a);
  EXPECT_EQ(b, _b);
  EXPECT_EQ(c, _c);
  EXPECT_EQ(d, _d);
}

TEST(Message, adds)
{
  const char greeting[12] = "helloworld!";
  const int life = 42;

  auto msg = Message<11 + sizeof(int)>{}.adds(greeting).add(life);
  static_assert(msg.size == 11 + sizeof(int), "The message has a wrong length!");
  std::stringstream ss;
  msg.write(ss);

  char echo[11];
  int is_life;
  Parser<11 + sizeof(int)>{}.add(echo).add(is_life).read(ss);
  EXPECT_EQ(std::string(&echo[0], 11), std::string(greeting));
  EXPECT_EQ(life, is_life);  // na-naaa naa na naa!
}

TEST(Message, encoding)
{
  unsigned char MSB = 0;
  char low = 'l';
  char high = 'h';
  auto message = Message<17>{}
                     .adds("\x80\x00\x0b\x80\x00\x0b")
                     .add(MSB)
                     .adds("\x01\x00")
                     .add(high)
                     .add(low)
                     .adds("\x80\x00\x0b\x80\x08\x0b");
  std::stringstream nss;
  message.write(nss);
  EXPECT_EQ(std::string("\x80\x00\x0b\x80\x00\x0b\x00\x01\x00hl\x80\x00\x0b\x80\x08\x0b", 17),
            nss.str());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
