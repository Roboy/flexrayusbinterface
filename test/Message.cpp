#include "flexrayusbinterface/Message.hpp"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
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


}

// Declare another test
TEST(Message, adds)
{
    const char greeting[12] = "helloworld!";
    const int life = 42;

    auto msg = Message<11 + sizeof(int)>{}.adds(greeting).add(life);
    static_assert(msg.size == 11+sizeof(int), "The message has a wrong length!");
    std::stringstream ss;
    msg.write(ss);

    char echo[11];
    int is_life;
    Parser<11>{}.add(echo).add(is_life).read(ss);
    EXPECT_EQ(std::string(&echo[0], 11), std::string(greeting));
    EXPECT_EQ(life, is_life); // na-naaa naa na naa!
    
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
