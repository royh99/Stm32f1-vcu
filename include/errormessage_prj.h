#ifndef ERRORMESSAGE_PRJ_H_INCLUDED
#define ERRORMESSAGE_PRJ_H_INCLUDED

#define ERROR_BUF_SIZE 4

#define ERROR_MESSAGE_LIST \
   ERROR_MESSAGE_ENTRY(BMSCOMM, ERROR_STOP) \
   ERROR_MESSAGE_ENTRY(OVERVOLTAGE, ERROR_STOP) \
   ERROR_MESSAGE_ENTRY(PRECHARGE, ERROR_STOP) \
   ERROR_MESSAGE_ENTRY(NONE, ERROR_DISPLAY) \
   ERROR_MESSAGE_ENTRY(THROTTLE1, ERROR_DISPLAY) \
   ERROR_MESSAGE_ENTRY(THROTTLE2, ERROR_DISPLAY) \
   ERROR_MESSAGE_ENTRY(THROTTLE12, ERROR_DISPLAY) \
   ERROR_MESSAGE_ENTRY(THROTTLE12DIFF, ERROR_DISPLAY) \
   ERROR_MESSAGE_ENTRY(THROTTLEMODE, ERROR_DISPLAY) \
   ERROR_MESSAGE_ENTRY(CANTIMEOUT, ERROR_DISPLAY) \
   ERROR_MESSAGE_ENTRY(TMPHSMAX, ERROR_DERATE) \
   ERROR_MESSAGE_ENTRY(TMPMMAX, ERROR_DERATE) \

#endif // ERRORMESSAGE_PRJ_H_INCLUDED
