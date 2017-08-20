#ifndef M_UTILS_H
#define M_UTILS_H

#include <time.h>


int m_get_parent_dir_name(char * buffer, size_t size);



/**
   @param day_string: "day" part of timestamp. NMEA timestamp only contains HHMMSS, and to build a full UNIX timestamp we need YYMMDD as well.
   @param nmea_ts - NMEA time stamp to convert
   @param ts_offset - offset of hours between local time zone and GPS's UTC

   @return time stamp in local time zone
*/
time_t m_nmea_gps_time_to_timestamp(const char * day_string, int nmea_ts, int ts_offset);



#endif /* #ifdef M_UTILS_H */
