#include "gmock/gmock.h"
#include "gtest/gtest.h"

extern "C" {
#include "timestamp.h"
}

static timestamp_t _timestamp = {.sec = 0, .usec = 0};

extern "C" timestamp_t timestamp() {
	return _timestamp;
}

class TimestampTest : public ::testing::Test {
	protected:
	void SetUp() {
	}

	void TearDown() {
	}
};

TEST_F(TimestampTest, check_timestamp) {
	_timestamp = {.sec = 1, .usec = 0};
	EXPECT_EQ(timestamp().sec, 1);
	_timestamp = {.sec = 10, .usec = 0};
	EXPECT_EQ(timestamp().sec, 10);
	_timestamp = {.sec = 0xfffffffe, .usec = 0};
	EXPECT_EQ(timestamp().sec, 4294967294);
}

TEST_F(TimestampTest, check_add_us) {
	timestamp_t ts = {.sec = 4, .usec = 3};
	timestamp_t t = timestamp_add_us(ts, 3000001UL);
	EXPECT_EQ(t.sec, 7);
	EXPECT_EQ(t.usec, 4);
}

TEST_F(TimestampTest, check_add_us_negative) {
	timestamp_t ts = {.sec = 4, .usec = 3};
	timestamp_t t;
	// these are invalid so we just record what is expected
	// a usec_t is uint32_t so max value is 4294_967295
	// this gives following values
	t = timestamp_add_us(ts, -3);
	EXPECT_EQ(t.sec, 4298); // 4294 + 4
	EXPECT_EQ(t.usec, 967293 + 3);
	t = timestamp_add_us(ts, -4);
	EXPECT_EQ(t.sec, 4298);
	EXPECT_EQ(t.usec, 967292 + 3);
}

TEST_F(TimestampTest, check_add_us_rollover) {
	timestamp_t ts = {.sec = 0xfffffffe, .usec = 0};
	timestamp_t t = timestamp_add_us(ts, 2000000UL);
	EXPECT_EQ(t.sec, 0);
	EXPECT_EQ(t.usec, 0);
	t = timestamp_add_us(ts, 3000000UL);
	EXPECT_EQ(t.sec, 1);
	EXPECT_EQ(t.usec, 0);
}

TEST_F(TimestampTest, check_sub) {
	timestamp_diff_t t = {.sec = 1, .usec = 4};
	timestamp_t a, b;
	a.sec = 2;
	a.usec = 200000;
	b.sec = 3;
	b.usec = 100000;
	t = timestamp_sub(a, b);
	EXPECT_EQ(t.sec, 0);
	EXPECT_EQ(t.usec, -900000);
	a.sec = 2;
	a.usec = 200000;
	b.sec = 2;
	b.usec = 100000;
	t = timestamp_sub(a, b);
	EXPECT_EQ(t.sec, 0);
	EXPECT_EQ(t.usec, 100000);
	a.sec = 3;
	a.usec = 200000;
	b.sec = 2;
	b.usec = 300000;
	t = timestamp_sub(a, b);
	EXPECT_EQ(t.sec, 0);
	EXPECT_EQ(t.usec, 900000);
	a.sec = 3;
	a.usec = 200000;
	b.sec = 2;
	b.usec = 100000;
	t = timestamp_sub(a, b);
	EXPECT_EQ(t.sec, 1);
	EXPECT_EQ(t.usec, 100000);
}

TEST_F(TimestampTest, check_expired) {
	_timestamp = {.sec = 10, .usec = 100};
	timestamp_t ts = {.sec = 4, .usec = 10};
	EXPECT_TRUE(timestamp_expired(ts));
	ts = {.sec = 10, .usec = 10};
	EXPECT_TRUE(timestamp_expired(ts));
	ts = {.sec = 10, .usec = 101};
	EXPECT_FALSE(timestamp_expired(ts));
	ts = {.sec = 11, .usec = 101};
	EXPECT_FALSE(timestamp_expired(ts));
	ts = {.sec = 10, .usec = 100};
	EXPECT_FALSE(timestamp_expired(ts));
}

TEST_F(TimestampTest, check_expired_past_limit) {
	_timestamp = {.sec = 0xfffffffd, .usec = 100};
	timestamp_t ts = {.sec = 0xfffffffe, .usec = 10};
	EXPECT_FALSE(timestamp_expired(ts));
	ts = {.sec = 0xfffffffc, .usec = 10};
	EXPECT_TRUE(timestamp_expired(ts));
}

TEST_F(TimestampTest, check_timestamp_before) {
	timestamp_t a, b;
	a.sec = 0;
	a.usec = 100;
	b.sec = 0;
	b.usec = 90;
	EXPECT_TRUE(timestamp_before(b, a));
	a.sec = 0;
	a.usec = 100;
	b.sec = 0;
	b.usec = 120;
	EXPECT_FALSE(timestamp_before(b, a));
	a.sec = 1;
	a.usec = 100;
	b.sec = 1;
	b.usec = 90;
	EXPECT_TRUE(timestamp_before(b, a));
	a.sec = 0;
	a.usec = 100;
	b.sec = 1;
	b.usec = 90;
	EXPECT_FALSE(timestamp_before(b, a));
}

int main(int argc, char **argv) {
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
