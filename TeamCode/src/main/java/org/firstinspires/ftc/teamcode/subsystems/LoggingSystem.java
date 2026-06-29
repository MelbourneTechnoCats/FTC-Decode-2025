

package org.firstinspires.ftc.teamcode.subsystems;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Simple file-based logging system for FTC.
 *
 * - Supports log levels: DEBUG, INFO, WARN, ERROR.
 * - Writes to a log file under app-internal storage.
 * - Path and file name are configurable.
 *
 * Usage:
 *   LoggingSystem.init(hardwareMap.appContext);
 *   LoggingSystem.setDirectoryName("logs");           // optional
 *   LoggingSystem.setFileName("robot-log.txt");       // optional
 *   LoggingSystem.i("DriveSubsystem", "Initialized");
 */
public class LoggingSystem {

    public enum LogLevel {
        DEBUG,
        INFO,
        WARN,
        ERROR
    }

    // Directory under Context.getFilesDir() where logs are stored
    private static String sDirectoryName = "logs";
    // Log file name within that directory
    private static String sFileName = "robot.log";

    // Minimum level to actually write to file
    private static LogLevel sMinLevel = LogLevel.DEBUG;

    private static File sLogFile;
    private static final Object sLock = new Object();
    private static final SimpleDateFormat sDateFormat =
            new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");

    private static boolean sInitialized = false;

    /**
     * Must be called once (e.g. in your first OpMode initialize()) before logging.
     *
     * Example:
     *   LoggingSystem.init(hardwareMap.appContext);
     */
    public static void init(Context appContext) {
        synchronized (sLock) {
            if (sInitialized) return;

            File baseDir = appContext.getFilesDir(); // internal app storage
            File logDir = new File(baseDir, sDirectoryName);
            if (!logDir.exists() && !logDir.mkdirs()) {
                RobotLog.ee("LoggingSystem", "Failed to create log directory: " + logDir.getAbsolutePath());
            }

            sLogFile = new File(logDir, sFileName);
            sInitialized = true;

            // Write header
            writeRawLine("========== LoggingSystem started: " +
                    sDateFormat.format(new Date()) + " ==========");
        }
    }

    /**
     * Set the directory name under app-internal storage.
     * Must be called before init() to take effect.
     */
    public static void setDirectoryName(String dirName) {
        synchronized (sLock) {
            if (sInitialized) {
                RobotLog.ww("LoggingSystem", "setDirectoryName() called after init(); ignoring.");
                return;
            }
            sDirectoryName = dirName;
        }
    }

    /**
     * Set the log file name.
     * Must be called before init() to take effect.
     */
    public static void setFileName(String fileName) {
        synchronized (sLock) {
            if (sInitialized) {
                RobotLog.ww("LoggingSystem", "setFileName() called after init(); ignoring.");
                return;
            }
            sFileName = fileName;
        }
    }

    /**
     * Set minimum level that will be written to file.
     */
    public static void setMinLevel(LogLevel level) {
        synchronized (sLock) {
            sMinLevel = level;
        }
    }

    public static void d(String tag, String msg) {
        log(LogLevel.DEBUG, tag, msg);
    }

    public static void i(String tag, String msg) {
        log(LogLevel.INFO, tag, msg);
    }

    public static void w(String tag, String msg) {
        log(LogLevel.WARN, tag, msg);
    }

    public static void e(String tag, String msg) {
        log(LogLevel.ERROR, tag, msg);
    }

    public static void e(String tag, String msg, Throwable t) {
        log(LogLevel.ERROR, tag, msg + " Exception: " + t.toString());
    }

    public static void log(LogLevel level, String tag, String msg) {
        synchronized (sLock) {
            if (!sInitialized || sLogFile == null) {
                // Fallback to RobotLog if not initialized
                RobotLog.ee("LoggingSystem", "Not initialized; message from " + tag + ": " + msg);
                return;
            }

            if (level.ordinal() < sMinLevel.ordinal()) {
                return;
            }

            String timestamp = sDateFormat.format(new Date());
            String prefix = "[" + level.name() + "]";
            String line = String.format("%s %s [%s] %s", timestamp, prefix, tag, msg);

            // Mirror to Logcat/RC log
            switch (level) {
                case DEBUG: RobotLog.dd(tag, msg); break;
                case INFO:  RobotLog.ii(tag, msg); break;
                case WARN:  RobotLog.ww(tag, msg); break;
                case ERROR: RobotLog.ee(tag, msg); break;
            }

            writeRawLine(line);
        }
    }

    private static void writeRawLine(String line) {
        if (sLogFile == null) return;
        FileWriter writer = null;
        try {
            writer = new FileWriter(sLogFile, true); // append
            writer.write(line);
            writer.write("\n");
            writer.flush();
        } catch (IOException e) {
            RobotLog.ee("LoggingSystem", "Failed to write log line: " + e.getMessage());
        } finally {
            if (writer != null) {
                try { writer.close(); } catch (IOException ignored) {}
            }
        }
    }

    /**
     * Get the absolute path of the current log file, for telemetry/debug.
     */
    public static String getLogFilePath() {
        synchronized (sLock) {
            return (sLogFile != null) ? sLogFile.getAbsolutePath() : "uninitialized";
        }
    }
}
