/*
 Copyright (C) 2020 Fredrik Öhrström

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include"dvparser.h"
#include"meters.h"
#include"meters_common_implementation.h"
#include"wmbus.h"
#include"wmbus_utils.h"
#include"string.h"

#define SOUNDER_FAULT_FLAG	   	0x0020
#define HEAD_TAMPER_FLAG		0x0040
#define EOL_REACHED_FLAG		0x0080
#define BATBIT0			0x0100
#define BATBIT1			0x0200
#define BATBIT2			0x0400
#define BATBIT3			0x0800
#define LOW_BAT_FAULT_FLAG		0x1000
#define HEAD_FAULT_FLAG		0x2000
#define PARITY_BIT_FLAG 		0x8000
#define ALARM_ACT_FLAG			0x10000
#define TEST_BTN_ACT_FLAG		0x20000
#define LOW_BAT_ACT_FAULT_FLAG     	0x40000
#define SENSOR_FAULT_FLAG		0x80000
#define GLITCH_DETECT_FLAG		0x100000
#define HEAD_CONNECT_FLAG		0x2000000
#define COMMSLINK_FLAG			0x40000000


struct MeterEI650FA : public virtual SmokeDetector, public virtual MeterCommonImplementation
{
    MeterEI650FA(MeterInfo &mi);

    string status();
    bool smokeDetected();
    string messageDate();
    string commissionDate();
    string lastSounderTestDate();
    string lastAlarmDate();
    string smokeAlarmCounter();
    string removedCounter();
    string totalRemoveDuration();
    string lastRemoveDate();
    string testButtonCounter();
    string testButtonLastDate();
    string sounderTestLastDate();
    string head_status();

private:

    void processContent(Telegram *t);

    private:

    string software_version_;
    string message_datetime_ {};
    uint8_t tpl_sts_ {};
    uint16_t info_codes_ {};
    uint32_t head_status_ {};
    uint16_t smoke_alarm_counter_ {};
    string commission_date_ {};
    string last_alarm_date_ {};
    uint32_t total_remove_duration_ {};
    string last_remove_date_ {};
    string test_button_last_date_ {};
    uint16_t removed_counter_ {};
    uint16_t test_button_counter_ {};
    string sounder_test_last_date_ {};

    map<int,string> error_codes_;
};

MeterEI650FA::MeterEI650FA(MeterInfo &mi) :
    MeterCommonImplementation(mi, MeterDriver::EI650FA)
{
    setExpectedTPLSecurityMode(TPLSecurityMode::AES_CBC_IV);

    addLinkMode(LinkMode::C1);

    // Vendor specific errors sent through the tpl status byte.
    /*
    error_codes_ = { { 0x10, "XXX" },
                     { 0x20, "YYY" },
                     { 0x30, "ZZZ" },
                     { 0xf0, "WWW" } };*/

    addPrint("software_version", Quantity::Text,
             [&](){ return software_version_; },
             "Software version.",
             false, true);
    addPrint("message_datetime", Quantity::Text,
             [&](){ return messageDate(); },
             "Date of message.",
             false, true);
    addPrint("commission_date", Quantity::Text,
             [&](){ return commissionDate(); },
             "Commission date",
             true, true);
    addPrint("last_alarm_date", Quantity::Text,
             [&](){ return lastAlarmDate(); },
             "Date of last alarm.",
             true, true);
    addPrint("smoke_alarm_counter", Quantity::Text,
             [&](){ return smokeAlarmCounter(); },
             "Number of times smoke alarm was triggered.",
             true, true);
    addPrint("total_remove_duration", Quantity::Text,
             [&](){ return totalRemoveDuration(); },
             "Number of times it was removed.",
             true, true);
    addPrint("last_remove_date", Quantity::Text,
             [&](){ return lastRemoveDate(); },
             "Date of last removal.",
             true, true);
    addPrint("removed_counter", Quantity::Text,
             [&](){ return removedCounter(); },
             "removed counter",
             true, true);
    addPrint("test_button_last_date", Quantity::Text,
             [&](){ return testButtonLastDate(); },
             "Date of last test button press.",
             true, true);
    addPrint("test_button_counter", Quantity::Text,
             [&](){ return testButtonCounter(); },
             "test button counter",
             true, true);
    addPrint("status", Quantity::Text,
             [&](){ return status(); },
             "Status of smoke detector.",
             true, true);
    addPrint("head status", Quantity::Text,
             [&](){ return head_status(); },
             "Head status of smoke detector.",
             true, true);
    addPrint("sounder_test_last_date", Quantity::Text,
             [&](){ return sounderTestLastDate(); },
             "Date of last sounder test.",
             true, true);             
}

shared_ptr<SmokeDetector> createEI650FA(MeterInfo &mi)
{
    return shared_ptr<SmokeDetector>(new MeterEI650FA(mi));
}

bool MeterEI650FA::smokeDetected()
{
    return 0;
}

void MeterEI650FA::processContent(Telegram *t)
{
    /*
      (EI650FA) 11: 0B dif (6 digit BCD Instantaneous value)
      (EI650FA) 12: FD vif (Second extension FD of VIF-codes)
      (EI650FA) 13: 0F vife (Software version #)
      (EI650FA) 14: * 060101 software version (1.1.6)
      (EI650FA) 17: 04 dif (32 Bit Integer/Binary Instantaneous value)
      (EI650FA) 18: 6D vif (Date and time type)
      (EI650FA) 19: * 300CAB22 message datetime (2021-02-11 12:48)
      (EI650FA) 1d: 02 dif (16 Bit Integer/Binary Instantaneous value)
      (EI650FA) 1e: FD vif (Second extension FD of VIF-codes)
      (EI650FA) 1f: 17 vife (Error flags (binary))
      (EI650FA) 20: * 0000 info codes (148ce5d8)
      (EI650FA) 22: 82 dif (16 Bit Integer/Binary Instantaneous value)
      (EI650FA) 23: 20 dife (subunit=0 tariff=2 storagenr=0)
      (EI650FA) 24: 6C vif (Date type G)
      (EI650FA) 25: AB22
      (EI650FA) 27: 42 dif (16 Bit Integer/Binary Instantaneous value storagenr=1)
      (EI650FA) 28: 6C vif (Date type G)
      (EI650FA) 29: 0101
      (EI650FA) 2b: 84 dif (32 Bit Integer/Binary Instantaneous value)
      (EI650FA) 2c: 40 dife (subunit=1 tariff=0 storagenr=0)
      (EI650FA) 2d: FF vif (Vendor extension)
      (EI650FA) 2e: 2C vife (per litre)
      (EI650FA) 2f: 000F1100
      (EI650FA) 33: 82 dif (16 Bit Integer/Binary Instantaneous value)
      (EI650FA) 34: 50 dife (subunit=1 tariff=1 storagenr=0)
      (EI650FA) 35: FD vif (Second extension FD of VIF-codes)
      (EI650FA) 36: 61 vife (Cumulation counter)
      (EI650FA) 37: * 0000 smoke alarm counter (0)
      (EI650FA) 39: 82 dif (16 Bit Integer/Binary Instantaneous value)
      (EI650FA) 3a: 50 dife (subunit=1 tariff=1 storagenr=0)
      (EI650FA) 3b: 6C vif (Date type G)
      (EI650FA) 3c: * 0101 last alarm date (2000-01-01)
      (EI650FA) 3e: 82 dif (16 Bit Integer/Binary Instantaneous value)
      (EI650FA) 3f: 60 dife (subunit=1 tariff=2 storagenr=0)
      (EI650FA) 40: FD vif (Second extension FD of VIF-codes)
      (EI650FA) 41: 61 vife (Cumulation counter)
      (EI650FA) 42: * 0000 removed counter (0)
      (EI650FA) 44: 83 dif (24 Bit Integer/Binary Instantaneous value)
      (EI650FA) 45: 60 dife (subunit=1 tariff=2 storagenr=0)
      (EI650FA) 46: FD vif (Second extension FD of VIF-codes)
      (EI650FA) 47: 31 vife (Duration of tariff [minute(s)])
      (EI650FA) 48: * 000000 total remove duration (0)
      (EI650FA) 4b: 82 dif (16 Bit Integer/Binary Instantaneous value)
      (EI650FA) 4c: 60 dife (subunit=1 tariff=2 storagenr=0)
      (EI650FA) 4d: 6C vif (Date type G)
      (EI650FA) 4e: * 0101 last remove date (2000-01-01)
      (EI650FA) 50: 82 dif (16 Bit Integer/Binary Instantaneous value)
      (EI650FA) 51: 70 dife (subunit=1 tariff=3 storagenr=0)
      (EI650FA) 52: FD vif (Second extension FD of VIF-codes)
      (EI650FA) 53: 61 vife (Cumulation counter)
      (EI650FA) 54: * 0100 test button counter (1)
      (EI650FA) 56: 82 dif (16 Bit Integer/Binary Instantaneous value)
      (EI650FA) 57: 70 dife (subunit=1 tariff=3 storagenr=0)
      (EI650FA) 58: 6C vif (Date type G)
      (EI650FA) 59: * AB22 test button last date (2021-02-11)
    */
    int offset;

    tpl_sts_ = t->tpl_sts;

    uint64_t serial;
    if (extractDVlong(&t->values, "0BFD0F", &offset, &serial))
    {
        // 060101 --> 01.01.06
        software_version_ =
            to_string((serial/10000)%100)+"."+
            to_string((serial/100)%100)+"."+
            to_string((serial)%100);
        t->addMoreExplanation(offset, " software version (%s)", software_version_.c_str());
    }

    struct tm datetime;
    if (extractDVdate(&t->values, "046D", &offset, &datetime))
    {
        message_datetime_ = strdatetime(&datetime);
        t->addMoreExplanation(offset, " message datetime (%s)", message_datetime_.c_str());
    }

    if (extractDVuint16(&t->values, "02FD17", &offset, &info_codes_))
    {
        string s = status();
        t->addMoreExplanation(offset, " info codes (%x)", s.c_str());
    }

    if (extractDVuint32(&t->values, "8440FF2C", &offset, &head_status_))
    {
        string hs = head_status();
        t->addMoreExplanation(offset, " head status (%x)", hs.c_str());
    }

    extractDVdate(&t->values, "82206C", &offset, &datetime);
    commission_date_ = strdate(&datetime);
    t->addMoreExplanation(offset, " commission date (%s)", commission_date_.c_str());

    extractDVdate(&t->values, "82506C", &offset, &datetime);
    last_alarm_date_ = strdate(&datetime);
    t->addMoreExplanation(offset, " last alarm date (%s)", last_alarm_date_.c_str());

    extractDVuint16(&t->values, "8450FD61", &offset, &smoke_alarm_counter_);
    t->addMoreExplanation(offset, " smoke alarm counter (%zu)", smoke_alarm_counter_);

    extractDVuint16(&t->values, "8460FD61", &offset, &removed_counter_);
    t->addMoreExplanation(offset, " removed counter (%zu)", removed_counter_);

    extractDVuint16(&t->values, "8470FD61", &offset, &test_button_counter_);
    t->addMoreExplanation(offset, " test button counter (%zu)", test_button_counter_);

    extractDVuint24(&t->values, "8360FD31", &offset, &total_remove_duration_);
    t->addMoreExplanation(offset, " total remove duration (%zu)", total_remove_duration_);

    extractDVdate(&t->values, "82606C", &offset, &datetime);
    last_remove_date_ = strdate(&datetime);
    t->addMoreExplanation(offset, " last remove date (%s)", last_remove_date_.c_str());

    extractDVdate(&t->values, "82706C", &offset, &datetime);
    test_button_last_date_ = strdate(&datetime);
    t->addMoreExplanation(offset, " test button last date (%s)", test_button_last_date_.c_str());
    
    extractDVdate(&t->values, "426C", &offset, &datetime);
    sounder_test_last_date_ = strdate(&datetime);
    t->addMoreExplanation(offset, " sounder test last date (%s)", sounder_test_last_date_.c_str());
}

string MeterEI650FA::status()
{
    string s = decodeTPLStatusByte(tpl_sts_, &error_codes_);

    if (s == "OK") s = ""; else s += " ";

    if (info_codes_ == 0) s.append("TPL Status Field ok ");

    if (s.length() > 0)
    {
        // There is something to report!
        s.pop_back(); // Remove final space
        return s;
    }
    return "OK";
}

string MeterEI650FA::messageDate()
{
	return message_datetime_;
}

string MeterEI650FA::commissionDate()
{
	return commission_date_;
}

string MeterEI650FA::lastAlarmDate()
{
	return last_alarm_date_;
}

string MeterEI650FA::totalRemoveDuration()
{
	return to_string(total_remove_duration_) + " minutes";
}

string MeterEI650FA::smokeAlarmCounter()
{
	return to_string(smoke_alarm_counter_);
}

string MeterEI650FA::testButtonCounter()
{
	return to_string(test_button_counter_);
}

string MeterEI650FA::removedCounter()
{
	return to_string(removed_counter_);
}

string MeterEI650FA::lastRemoveDate()
{
	return last_remove_date_;
}

string MeterEI650FA::testButtonLastDate()
{
	return test_button_last_date_;
}

string MeterEI650FA::sounderTestLastDate()
{
	return sounder_test_last_date_;
}

string MeterEI650FA::head_status()
{
    string hs = decodeTPLStatusByte(tpl_sts_, &error_codes_);

    if (hs == "OK") hs = ""; else hs += " ";

    if (head_status_ & HEAD_TAMPER_FLAG) hs.append("HEAD TAMPER FLAG Set ");
    if (head_status_ & SOUNDER_FAULT_FLAG) hs.append("SOUNDER FAULT FLAG Set ");
    if (head_status_ & LOW_BAT_FAULT_FLAG) hs.append("LOW BATTERY FLAG Set ");
    if (head_status_ & EOL_REACHED_FLAG) hs.append("EOL REACHED FLAG Set ");
    if (head_status_ & HEAD_FAULT_FLAG) hs.append("HEAD FAULT FLAG Set ");
    if (head_status_ & PARITY_BIT_FLAG) hs.append("PARITY BIT FLAG Set ");
    if (head_status_ & GLITCH_DETECT_FLAG) hs.append("GLITCH DETECT FLAG Set ");
   
 
    if (hs.length() > 0)
    {
        // There is something to report!
        hs.pop_back(); // Remove final space
        return hs;
    }
    return "OK";
}
