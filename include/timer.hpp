#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost::asio;
using namespace std;

class Deadline 
{
public:
    Deadline(deadline_timer &timer) : t(timer) {
        wait();
    }

    void timeout(const boost::system::error_code &e) {
        if (e)
            return;
        cout << "tick" << endl;
        wait();
    }

    void cancel() {
        t.cancel();
    }


private:
    void wait() {
        t.expires_from_now(boost::posix_time::seconds(1)); //repeat rate here
        t.async_wait(boost::bind(&Deadline::timeout, this, boost::asio::placeholders::error));
    }

    deadline_timer &t;
};


class CancelDeadline {
public:
    CancelDeadline(Deadline &d) :dl(d) { }
    void operator()() {
        string cancel;
        cin >> cancel;
        dl.cancel();
        return;
    }
private:
    Deadline &dl;
};
