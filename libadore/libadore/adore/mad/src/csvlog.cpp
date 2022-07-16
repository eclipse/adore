/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Robert Markowski - initial API and implementation
 ********************************************************************************/

#include <adore/mad/csvlog.h>

#define MULTIPLE_ARGUMENT_HANDLER char outbuf[1024]; \
	va_list arg_list; \
	va_start(arg_list, format); \
	vsprintf(outbuf, format, arg_list); \
	va_end(arg_list)

//PUBLIC --------------------------------------------------------------------------------
	//    -------------------------------------------------------------------------------
CSVLog& CSVLog::getInstance()
{
	static CSVLog _instance;
	return _instance;
}

//SET -------------------------------------------------------------------------------
//    -------------------------------------------------------------------------------
void CSVLog::setLogLevel(LogLevel lvl)
{
	getCurrentLogLevelFile() = lvl;
	getCurrentLogLevelConsole() = lvl;
}

void CSVLog::setLogLevelFile(LogLevel lvl)
{
	getCurrentLogLevelFile() = lvl;
}

void CSVLog::setLogLevelConsole(LogLevel lvl)
{
	getCurrentLogLevelConsole() = lvl;
}

void CSVLog::setTimestampVersion(CSVLog::TimestampVersion tsv)
{
	_timestampVersion = tsv;
}

void CSVLog::setMaxLinesFile(int numLines)
{
	_maxLinesFile = numLines;
}

void CSVLog::setMaxLinesBuffer(int numLines)
{
	_maxLinesBuffer = numLines;
}

void CSVLog::setSeperator(char seperator)
{
	_seperator = seperator;
}

void CSVLog::setLowThroughputMode(bool b)
{
	_lowThroughputMode = b;
}

void CSVLog::setTimeBetweenWrites(double time_seconds)
{
	_maxTimeBetweenWrites = time_seconds;
	setLowThroughputMode(true);
}

//GET -------------------------------------------------------------------------------
//    -------------------------------------------------------------------------------

CSVLog::LogLevel& CSVLog::getCurrentLogLevelFile()
{
	static LogLevel _logLevel = _TRACE;
	return _logLevel;
}

CSVLog::LogLevel& CSVLog::getCurrentLogLevelConsole()
{
	static LogLevel _logLevel = _ERROR;
	return _logLevel;
}

bool CSVLog::hasFile()
{
	return _useFile;
}

//LOG -------------------------------------------------------------------------------
//    -------------------------------------------------------------------------------
void CSVLog::logTrace(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	writeLog(outbuf, _TRACE);
}
void CSVLog::logTraceConsole(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	logConsole(outbuf,_TRACE);
}
void CSVLog::logTraceFile(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	logFile(outbuf,_TRACE);
}

void CSVLog::logInfo(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	writeLog(outbuf, _INFO);
}
void CSVLog::logInfoConsole(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	logConsole(outbuf,_INFO);
}
void CSVLog::logInfoFile(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	logFile(outbuf,_INFO);
}

void CSVLog::logWarning(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	writeLog(outbuf, _WARNING);
}
void CSVLog::logWarningConsole(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	logConsole(outbuf,_WARNING);
}
void CSVLog::logWarningFile(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	logFile(outbuf,_WARNING);
}

void CSVLog::logError(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	writeLog(outbuf, _ERROR);
}
void CSVLog::logErrorConsole(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	logConsole(outbuf,_ERROR);
}
void CSVLog::logErrorFile(const char *format, ...)
{
	MULTIPLE_ARGUMENT_HANDLER; //now available: input arguments in message as 'outbuf'
	logFile(outbuf,_ERROR);
}

//PRIVATE -------------------------------------------------------------------------------
	//    -------------------------------------------------------------------------------
CSVLog::CSVLog()
{
	_linecountBuffer = 0;
	_linecountFile = 0;
	_maxLinesBuffer = 1000;
	_maxLinesFile = 100 * _maxLinesBuffer;
	_timestampVersion = _TIMEOFDAY;
	_filename = "";
	_seperator = '\0';
	_useFile = false;
	_lastWriteTime = std::clock();
	_maxTimeBetweenWrites = 5.0;
	_lowThroughputMode = true;

}

CSVLog::~CSVLog()
{
	if (_linecountBuffer > 0)
		writeBufferToFile();
}

void CSVLog::init(std::string filename)
{
	_filename = filename;
	if(_filename.length()>0)
	{
		_useFile = true;
	}
	else
	{
		_useFile = false;
	}
	if (_seperator == '\0')
		_seperator = ',';
}

//WRITE -----------------------------------------------------------------------------
//    -------------------------------------------------------------------------------
void CSVLog::writeLog(std::string msg, CSVLog::LogLevel lvl)
{
	std::string time;
	if (_timestampVersion == _TIMEOFDAY)
		time = getCurrentTime();
	else if (_timestampVersion == _EPOCH)
		time = getCurrentTimeEpoch();

	if (_filename.length() > 0 && lvl >= getCurrentLogLevelFile())
	{
		writeLogToBuffer(time, msg, lvl);

		if (_linecountBuffer >= _maxLinesBuffer)
			writeBufferToFile();
	}

	if (lvl >= getCurrentLogLevelConsole())
		writeLogToConsole(time, msg, lvl);
}

void CSVLog::logConsole(std::string msg, CSVLog::LogLevel lvl)
{
	std::string time;
	if (_timestampVersion == _TIMEOFDAY)
	{
		time = getCurrentTime();
	}
	else if (_timestampVersion == _EPOCH)
	{
		time = getCurrentTimeEpoch();
	}
	writeLogToConsole(time, msg, lvl);
}
void CSVLog::logFile(std::string msg, CSVLog::LogLevel lvl)
{
	std::string time;
	if (_timestampVersion == _TIMEOFDAY)
		time = getCurrentTime();
	else if (_timestampVersion == _EPOCH)
		time = getCurrentTimeEpoch();

	if (_useFile)
	{
		writeLogToBuffer(time, msg, lvl);

		if (_linecountBuffer >= _maxLinesBuffer ||
			(_lowThroughputMode && _linecountBuffer > 0 && (std::clock() - _lastWriteTime) / (double) CLOCKS_PER_SEC > _maxTimeBetweenWrites)
			)
		{
			writeBufferToFile();
			_lastWriteTime = std::clock();
		}

	}
}

void CSVLog::writeBufferToFile()
{
	if (_linecountFile >= _maxLinesFile || _currentFilename.empty())
	{
		_currentFilename = _filename + "-" + getCurrentTimeEpoch() + ".log";
		_linecountFile = 0;
	}
	FILE* f = fopen(_currentFilename.c_str(), "a");
	fprintf(f, "%s", os.str().c_str());
	fflush(f);
	fclose(f);
	os.str(std::string());
	_linecountFile += _linecountBuffer;
	_linecountBuffer = 0;
}

void CSVLog::writeLogToBuffer(std::string time, std::string msg, CSVLog::LogLevel lvl)
{
	os << time << _seperator << lvlToString(lvl) << _seperator << msg << std::endl;
	_linecountBuffer++;
}

void CSVLog::setLogLevelFromConsole()
{
	std::printf("Set new log level: ");
	char i;
	std::cin.clear();
	std::cin>>i;
	std::cin.clear();
	switch(i)
	{
	case 'E':
		LOG_SET_LVL_E();
		LOG_E("Set log level to Error");
		break;
	case 'W':
		LOG_SET_LVL_W();
		LOG_W("Set log level to Warning");
		break;
	case 'I':
		LOG_SET_LVL_I();
		LOG_I("Set log level to Info");
		break;
	case 'T':
		LOG_SET_LVL_T();
		LOG_T("Set log level to Trace");
		break;
	default:
		LOG_E("%s","Oops, that didn't work! Try again with E for Error, W for Warning, I for Info or T for Trace");
		break;
	}
}

//CONVERT ---------------------------------------------------------------------------
//    -------------------------------------------------------------------------------
std::string CSVLog::lvlToString(CSVLog::LogLevel lvl)
{
	static const char* const buffer[] = { "TRACE", "INFO", "WARNING", "ERROR" };
	return buffer[lvl];
}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)

#include <Windows.h>

std::string CSVLog::getCurrentTime()
{
	const int MAX_LEN = 200;
	char buffer[MAX_LEN];
	if (GetTimeFormatA(LOCALE_USER_DEFAULT, 0, 0,
		"HH':'mm':'ss", buffer, MAX_LEN) == 0)
		return "Error in NowTime()";

	char result[100] = { 0 };
	SYSTEMTIME st;
	GetSystemTime(&st);
	sprintf_s(result, "%s.%03ld", buffer, (long)(st.wMilliseconds));
	//sprintf_s(result, "%s.%03ld", buffer, (long)(GetTickCount()%1000));
	return result;
}

std::string CSVLog::getCurrentTimeEpoch()
{
	char result[100] = { 0 };
	SYSTEMTIME st;
	GetSystemTime(&st);
	std::time_t t = std::time(nullptr);
	sprintf(result, "%ld%03ld", (long)(t), (long)(st.wMilliseconds));
	return result;
}

void CSVLog::writeLogToConsole(std::string time, std::string msg, CSVLog::LogLevel lvl)
{
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), static_cast<WORD>(lvlToConsoleColor(lvl)));
	std::cout << time << " " << lvlToString(lvl) << ":\t" << msg << std::endl;
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
}

int CSVLog::lvlToConsoleColor(CSVLog::LogLevel lvl)
{
	//							TRACE	INFO	WARNING	ERROR
	//							white	green	yellow	red
	static const int buffer[] = { 15,	10,		14,		12 };
	return buffer[lvl];
}

#else

#include <sys/time.h>

std::string CSVLog::getCurrentTime()
{
	char buffer[11];
	time_t t;
	time(&t);
	tm r = { 0 };
	strftime(buffer, sizeof(buffer), "%X", localtime_r(&t, &r));
	struct timeval tv;
	gettimeofday(&tv, 0);
	char result[100] = { 0 };
	std::sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000);
	return result;
}

std::string CSVLog::getCurrentTimeEpoch()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	char result[100] = { 0 };
	std::sprintf(result, "%lu%ld", tv.tv_sec, tv.tv_usec);
	return result;
}

void CSVLog::writeLogToConsole(std::string time, std::string msg, CSVLog::LogLevel lvl)
{
	std::cout << lvlToConsoleColorLinux(lvl) << time << " " << lvlToString(lvl) << ":\t" << msg << "\033[0m" << std::endl;
}

std::string CSVLog::lvlToConsoleColorLinux(CSVLog::LogLevel lvl)
{
	//										TRACE		INFO		WARNING		ERROR
	//										white		green		yellow		red
	static const char* const buffer[] = { "\033[37m", "\033[32m", "\033[33m", "\033[31m" };
	return buffer[lvl];
}

#endif //WIN32