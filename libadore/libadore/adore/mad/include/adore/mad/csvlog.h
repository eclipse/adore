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

#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <ctime>

/**
 * @brief log on trace level
 * 
 */
#define LOG_T(...)	if (CSVLog::getCurrentLogLevelConsole()<=CSVLog::_TRACE) \
							CSVLog::getInstance().logTraceConsole(__VA_ARGS__); \
					if (CSVLog::getInstance().hasFile() && CSVLog::getCurrentLogLevelFile()<=CSVLog::_TRACE) \
							CSVLog::getInstance().logTraceFile(__VA_ARGS__)

/**
 * @brief log on info level
 * 
 */
#define LOG_I(...)	if (CSVLog::getCurrentLogLevelConsole()<=CSVLog::_INFO) \
							CSVLog::getInstance().logInfoConsole(__VA_ARGS__); \
					if (CSVLog::getInstance().hasFile() && CSVLog::getCurrentLogLevelFile()<=CSVLog::_INFO) \
							CSVLog::getInstance().logInfoFile(__VA_ARGS__)

/**
 * @brief log on warning level
 * 
 */
#define LOG_W(...)	if (CSVLog::getCurrentLogLevelConsole()<=CSVLog::_WARNING) \
							CSVLog::getInstance().logWarningConsole(__VA_ARGS__); \
					if (CSVLog::getInstance().hasFile() && CSVLog::getCurrentLogLevelFile()<=CSVLog::_WARNING) \
							CSVLog::getInstance().logWarningFile(__VA_ARGS__)

/**
 * @brief log on error level
 * 
 */
#define LOG_E(...)	if (CSVLog::getCurrentLogLevelConsole()<=CSVLog::_ERROR) \
							CSVLog::getInstance().logErrorConsole(__VA_ARGS__); \
					if (CSVLog::getInstance().hasFile() && CSVLog::getCurrentLogLevelFile()<=CSVLog::_ERROR) \
							CSVLog::getInstance().logErrorFile(__VA_ARGS__)

/**
 * @brief change log level via command line input
 * 
 */
#define LOG_SET_LVL_FROM_CONSOLE(...) CSVLog::getInstance().setLogLevelFromConsole()

/**
 * @brief set log level to trace
 * 
 */
#define LOG_SET_LVL_T(...) CSVLog::getInstance().setLogLevel(CSVLog::_TRACE)

/**
 * @brief set log level to info
 * 
 */
#define LOG_SET_LVL_I(...) CSVLog::getInstance().setLogLevel(CSVLog::_INFO)

/**
 * @brief set log level to warning
 * 
 */
#define LOG_SET_LVL_W(...) CSVLog::getInstance().setLogLevel(CSVLog::_WARNING)

/**
 * @brief set log level to error
 * 
 */
#define LOG_SET_LVL_E(...) CSVLog::getInstance().setLogLevel(CSVLog::_ERROR)

/**
 * @brief set log level of console to trace
 * 
 */
#define LOG_SET_LVL_CONSOLE_T(...) CSVLog::getInstance().setLogLevelConsole(CSVLog::_TRACE)

/**
 * @brief set log level of console to info
 * 
 */
#define LOG_SET_LVL_CONSOLE_I(...) CSVLog::getInstance().setLogLevelConsole(CSVLog::_INFO)

/**
 * @brief set log level of console to warning
 * 
 */
#define LOG_SET_LVL_CONSOLE_W(...) CSVLog::getInstance().setLogLevelConsole(CSVLog::_WARNING)

/**
 * @brief set log level of console to error
 * 
 */
#define LOG_SET_LVL_CONSOLE_E(...) CSVLog::getInstance().setLogLevelConsole(CSVLog::_ERROR)

/**
 * @brief set log level of file to trace
 * 
 */
#define LOG_SET_LVL_FILE_T(...) CSVLog::getInstance().setLogLevelFile(CSVLog::_TRACE)

/**
 * @brief set log level of file to info
 * 
 */
#define LOG_SET_LVL_FILE_I(...) CSVLog::getInstance().setLogLevelFile(CSVLog::_INFO)

/**
 * @brief set log level of file to warning
 * 
 */
#define LOG_SET_LVL_FILE_W(...) CSVLog::getInstance().setLogLevelFile(CSVLog::_WARNING)

/**
 * @brief set log level of file to error
 * 
 */
#define LOG_SET_LVL_FILE_E(...) CSVLog::getInstance().setLogLevelFile(CSVLog::_ERROR)


/**
 * @brief set seperator for file log
 * 
 */
#define LOG_SET_SEPERATOR(char_seperator) CSVLog::getInstance().setSeperator(char_seperator)

/**
 * @brief set max lines per file before a new file is generated
 * 
 */
#define LOG_SET_MAXLINES_FILE(int_maxLines) CSVLog::getInstance().setMaxLinesFile(int_maxLines)

/**
 * @brief set max lines in buffer before buffer is written to file
 * 
 */
#define LOG_SET_MAXLINES_BUFFER(int_maxLines) CSVLog::getInstance().setMaxLinesBuffer(int_maxLines)

/**
 * @brief set time stamp version (0 for time of day or 1 for epoch)
 * 
 */
#define LOG_SET_TIMESTAMPVERSION(timestampversion_tsv) CSVLog::getInstance().setTimestampVersion(timestampversion_tsv)

/**
 * @brief in low throughput mode, the buffer is written to the file after a certain maximum time even if the buffer is not full
 * 
 */
#define LOG_SET_LOWTHROUGHPUTMODE(bool_active) CSVLog::getInstance().setLowThroughputMode(bool_active)

/**
 * @brief maximum time between writing the buffer to the file in low throughput mode
 * 
 */
#define LOG_SET_MAXTIMEBETWEENWRITES(double_time_seconds) CSVLog::getInstance().setTimeBetweenWrites(double_time_seconds)

/**
 * @brief initiates writing to a file
 * 
 */
#define LOG_INIT(string_filename) CSVLog::getInstance().init(string_filename)

/**
 * @brief singleton class for logging on the console and in a file, only interact with it via the defined macros
 * 
 */
class CSVLog
{
public:
	/**
	 * @brief Construct a new CSVLog object
	 * 
	 */
	CSVLog();

	/**
	 * @brief Destroy the CSVLog object
	 * 
	 */
	virtual ~CSVLog();

	/**
	 * @brief Get the singleton object of CSVLog
	 * 
	 * @return CSVLog& 
	 */
	static CSVLog& getInstance();

	enum LogLevel{
		_TRACE,
		_INFO,
		_WARNING,
		_ERROR
	};

	enum TimestampVersion{
		_TIMEOFDAY,
		_EPOCH
	};
	
	/**
	 * @brief initialize log to file
	 * 
	 * @param filename 
	 */
	void init(std::string filename);

	/**
	 * @brief set the log level for file and console
	 * 
	 * @param lvl 
	 */
	void setLogLevel(LogLevel lvl);

	/**
	 * @brief set the log level via console input
	 * 
	 */
	void setLogLevelFromConsole();

	/**
	 * @brief set log level only for file
	 * 
	 * @param lvl 
	 */
	void setLogLevelFile(LogLevel lvl);

	/**
	 * @brief set log level only for console
	 * 
	 * @param lvl 
	 */
	void setLogLevelConsole(LogLevel lvl);

	/**
	 * @brief set the time stamp version, time of day (0) or epoch (1)
	 * 
	 * @param tsv 
	 */
	void setTimestampVersion(TimestampVersion tsv);

	/**
	 * @brief set seperator for log in file
	 * 
	 * @param seperator 
	 */
	void setSeperator(char seperator);

	/**
	 * @brief defines at how many lines in the buffer the buffer is written into a file
	 * 
	 * @param numLines 
	 */
	void setMaxLinesBuffer(int numLines);

	/**
	 * @brief define after how many lines a new file is created
	 * 
	 * @param numLines 
	 */
	void setMaxLinesFile(int numLines);

	/**
	 * @brief in low throughput mode, the buffer is written to the file after a certain time even if the max buffer size is not reached yet
	 * 
	 * @param b 
	 */
	void setLowThroughputMode(bool b);

	/**
	 * @brief defines the max time between the buffer being written to the file in low throughput mode
	 * 
	 * @param time_seconds 
	 */
	void setTimeBetweenWrites(double time_seconds);

	/**
	 * @brief get current log level for file log
	 * 
	 * @return LogLevel& 
	 */
	static LogLevel& getCurrentLogLevelFile();

	/**
	 * @brief get current log level for console log
	 * 
	 * @return LogLevel& 
	 */
	static LogLevel& getCurrentLogLevelConsole();
	
	/**
	 * @brief general log on trace level
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logTrace(const char *format, ...);

	/**
	 * @brief log on trace level in console
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logTraceConsole(const char *format, ...);

	/**
	 * @brief log on trace level in file
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logTraceFile(const char *format, ...);

	/**
	 * @brief general log on info level
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logInfo(const char *format, ...);

	/**
	 * @brief log on info level in console
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logInfoConsole(const char *format, ...);

	/**
	 * @brief log on info level in file
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logInfoFile(const char *format, ...);

	/**
	 * @brief general log on warning level
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logWarning(const char *format, ...);

	/**
	 * @brief log on warning level in console
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logWarningConsole(const char *format, ...);

	/**
	 * @brief log on warning level in file
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logWarningFile(const char *format, ...);

	/**
	 * @brief general log on error level
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logError(const char *format, ...);

	/**
	 * @brief log on error level in console
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logErrorConsole(const char *format, ...);

	/**
	 * @brief log on error level in file
	 * 
	 * @param format 
	 * @param ... 
	 */
	void logErrorFile(const char *format, ...);

	/**
	 * @brief check whether a file is available
	 * 
	 * @return true 
	 * @return false 
	 */
	bool hasFile();

private:
	CSVLog(const CSVLog&);
	void operator=(CSVLog const&);

	char _seperator;
	int _linecountBuffer;
	int _linecountFile;
	int _maxLinesBuffer;
	int _maxLinesFile;
	double _maxTimeBetweenWrites;
	bool _useFile;
	bool _lowThroughputMode;
	TimestampVersion _timestampVersion;
	std::string _filename;
	std::string _currentFilename;
	std::ostringstream os;

	std::clock_t _lastWriteTime;
	
	/**
	 * @brief general log generator method
	 * 
	 * @param msg 
	 * @param lvl 
	 */
	void writeLog(std::string msg, LogLevel lvl);	

	/**
	 * @brief general log generator for console
	 * 
	 * @param time 
	 * @param msg 
	 * @param lvl 
	 */
	void writeLogToConsole(std::string time, std::string msg, LogLevel lvl);

	/**
	 * @brief general log generator for buffer
	 * 
	 * @param time 
	 * @param msg 
	 * @param lvl 
	 */
	void writeLogToBuffer(std::string time, std::string msg, LogLevel lvl);

	/**
	 * @brief writes content of buffer to file
	 * 
	 */
	void writeBufferToFile();

	/**
	 * @brief generate log entry in console
	 * 
	 * @param msg 
	 * @param lvl 
	 */
	void logConsole(std::string msg, LogLevel lvl);

	/**
	 * @brief generate log entry in file
	 * 
	 * @param msg 
	 * @param lvl 
	 */
	void logFile(std::string msg, LogLevel lvl);

	/**
	 * @brief get time of day as string
	 * 
	 * @return std::string 
	 */
	std::string getCurrentTime();

	/**
	 * @brief get epoch as string
	 * 
	 * @return std::string 
	 */
	std::string getCurrentTimeEpoch();

	/**
	 * @brief get string for log level
	 * 
	 * @param lvl 
	 * @return std::string 
	 */
	std::string lvlToString(LogLevel lvl);

	/**
	 * @brief get console color from log level on windows
	 * 
	 * @param lvl 
	 * @return int 
	 */
	int lvlToConsoleColor(LogLevel lvl);

	/**
	 * @brief get console color from log level on linux
	 * 
	 * @param lvl 
	 * @return std::string 
	 */
	std::string lvlToConsoleColorLinux(LogLevel lvl);
};