// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/json.hpp>
#include <connect/authentication.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace connect_plugins {
class ExampleAuthentication final : public authentication::Authentication {
   public:
    /**
     * Retrieves the authentication information given a user key
     *
     * @param logger logger to log with
     * @param endpoint endpoint to authenticate against, should start with "/"
     * @param host host to authenticate against
     * @param port port to authenticate against
     * @param ssl if ssl should be used to authentication
     * @param userKey user key to resolve
     * @param nameSpace the namespace this is running in
     * @param domainId the domain id this is running in
     * @return true if a valid authentication information was retrieved, false otherwise
     */
    bool getAuthenticationFromUserKey(const Logger &logger, const std::string &endpoint, const std::string &host, const std::string &port, const bool ssl, const std::string &userKey, const std::string &nameSpace, const int64_t domainId) override {
        // simply set some mockup data
        this->setEnd(parse_iso8601_to_utc("2026-02-04T23:59:59Z"));
        this->setUser("user@email.de");
        return true;
    }

   private:
    /**
     * This parses a time-string conform to ISO8601 into a UTC posix_time.
     * This supports especially the following ISO8601 formats:
     *  - "2025-02-18T23:09:00.000Z"
     *  - "2025-02-18T23:09:00Z"
     *  - "2025-02-18T23:09:00.000+01:00"
     *  - "2025-02-18T23:09:00+01:30"
     *  - "2025-02-18T23:09:00-05:00"
     *
     * This is necessary since boost::posix_time does only partially support ISO8601.
     * It especially does not support time zone specifier.
     *
     * @param iso8601 a time conform to ISO8601
     * @return a posix_time::ptime in UTC
     */
    static boost::posix_time::ptime parse_iso8601_to_utc(const std::string &iso8601) {
        // first we need to separate the main date-time portion from the time-zone
        std::size_t offsetPos = std::string::npos;

        // check for trailing 'Z'
        offsetPos = iso8601.rfind('Z');
        // no trailing 'Z' found ... look for '+' or '-' near the end
        if (offsetPos == std::string::npos) {
            const std::size_t plusPos = iso8601.rfind('+');
            const std::size_t minusPos = iso8601.rfind('-');

            // no offset symbol at all => no time zone offset
            if (plusPos == std::string::npos && minusPos == std::string::npos)
                offsetPos = std::string::npos;
            else {
                // we have a minus
                if (plusPos == std::string::npos) offsetPos = minusPos;
                // we have a plus
                else if (minusPos == std::string::npos)
                    offsetPos = plusPos;
                // we have both, pick whichever is larger index (we need to do this since the timestamp itself always contains '-' as separator)
                else
                    offsetPos = (plusPos > minusPos) ? plusPos : minusPos;
            }
        }

        // now extract the date-time and offset part
        std::string dateTimePart;
        std::string offsetPart;

        if (offsetPos != std::string::npos) {
            dateTimePart = iso8601.substr(0, offsetPos);
            offsetPart = iso8601.substr(offsetPos);
        } else
            dateTimePart = iso8601;

        // now we parse the date-time part into a ptime
        // this is done using the from_iso_extended_string which should be able to parse the date-time part we are having
        // if this stops working in the future, we can manually transform the date-time part and use a time_input_facet
        const boost::posix_time::ptime localPTime = boost::posix_time::from_iso_extended_string(dateTimePart);
        if (localPTime.is_not_a_date_time()) throw std::invalid_argument("Failed to parse date-time part: " + dateTimePart);

        // now we parse the offset part and adjust the localPTime to get UTC
        // the offset part can be:
        //   "Z"
        //   "+HH"
        //   "+HH:MM"
        //   "-HH"
        //   "-HH:MM"
        boost::posix_time::time_duration offsetTD;  // how much we must *subtract* from local time to get UTC
        // we are already in UTC
        if (offsetPart.empty())
            offsetTD = boost::posix_time::time_duration(0, 0, 0);
        else {
            // we are already in UTC
            if (offsetPart == "Z")
                offsetTD = boost::posix_time::time_duration(0, 0, 0);
            else {
                // extract the sign
                if (offsetPart[0] != '+' && offsetPart[0] != '-') throw std::invalid_argument("Offset must begin with '+' or '-' or be 'Z'. Found: " + offsetPart);
                const int sign = (offsetPart[0] == '+') ? +1 : -1;

                // remove the sign
                std::string hhmm = offsetPart.substr(1);

                int hours = 0;
                int minutes = 0;

                // find colon (hours:minutes) if any
                const std::size_t colonPos = hhmm.find(':');
                // no colon ... just hours
                if (colonPos == std::string::npos)
                    hours = std::stoi(hhmm);
                else {
                    hours = std::stoi(hhmm.substr(0, colonPos));
                    minutes = std::stoi(hhmm.substr(colonPos + 1));
                }

                // compute the offset as time_duration
                offsetTD = boost::posix_time::hours(hours) + boost::posix_time::minutes(minutes);
                if (sign < 0) offsetTD = -offsetTD;
            }
        }

        // finally compute the UTC time as localPTime minus (or plus) the extracted offset
        // the offsetTD will be negative for a '-' offset and positive for a '+' offset
        // hence if the offset was e.g. -01:00 we must add an hour to get UTC
        //       if the offset was e.g. +01:00 we must subtract an hour get get UTC
        const boost::posix_time::ptime utcTime = localPTime - offsetTD;
        return utcTime;
    }
};

}  // namespace connect_plugins

PLUGINLIB_EXPORT_CLASS(connect_plugins::ExampleAuthentication, authentication::Authentication)