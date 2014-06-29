#ifndef __FLIGHT_CONTROLLER_H
#define __FLIGHT_CONTROLLER_H

#include <QString>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "globals.hpp"

/**
 * @brief Klasse die das Singleton-Designpattern<BR>
 * anwendet und für die Steuerung der Drohne zuständig ist.
 */
class FlightController
{
private:

    /**
     * @brief Ist für die Verbindung, eines eigenen<BR>
     * sogenannten "Nodes", zur ROS zuständig
     */
    ros::NodeHandle *m_FlightControllerNodeHandle;

    /**
     * @brief Übergibt Befehle an ROS, um die Drohne zu steuern
     */
    ros::Publisher m_FlightControllerPublisher;

    /**
     * @brief Leiter an ROS den Befehl,<BR>
     * dass die Drohne abheben soll, weiter
     */
    ros::Publisher m_TakeOffPublisher;

    /**
     * @brief Gibt ROS die Aufgabe,<BR>
     * die Drohne wieder landen zu lassen
     */
    ros::Publisher m_LandPublisher;

    /**
     * @brief Mutex um die Thread-Sicherheit zu gewähren
     */
    std::mutex m_ThreadMutex;

    /**
     * @brief Thread, in dem die FlightController-Instanz<BR>
     * ausgeführt wird. Genauer gesagt, in welchem die "run"-Methode<BR>
     * ausgeführt wird, welche für die Befehlsübergae<BR>
     * an ROS zuständig ist.
     * @see run
     */
    std::thread *m_Thread;

    /**
     * @brief Gibt an, ob der Thread gestoppt werden soll.<BR>
     * Verhindert weitere Befehlsübergabe an ROS
     */
    std::atomic<bool> m_Stop;

    /**
     * @brief Gibt an, ob der letzte Befehl ausgeführt werden soll
     */
    std::atomic<bool> m_RunLastCommand;

    /**
     * @brief Stellt den zuvpr ausgeführten Befehl dar
     */
    geometry_msgs::Twist m_LastCommand;

    /**
     * @brief Aktuelle Art der Befehlsausführung
     * @see Globals::CommandType
     */
    Globals::CommandType m_CurrentCommandType;

    /**
     * @brief Frequenz, in welcher Zeit ein Kommando ausgeführt wird
     */
    int m_Frequency;

    /**
     * @brief "Kommando-Zentrale"
     */
    void run();

    /**
     * @brief Konstruktor zum Erzeugen der FlightController-Instanz.<BR>
     * Aufgrund des Singleton-Designpatterns: private
     * @param p_Argc Anzahl der Argumente - für ROS
     * @param p_Argv Argumente - für ROS
     */
    FlightController(int p_Argc, char **p_Argv);

    /**
     * @brief Standard-Kopierkonstruktor.<BR>
     * private und keine Aufgabe, aufgrund des Singleton-Designpatterns
     */
    FlightController(FlightController const&) = delete;

    /**
     * @brief Überladener Zuweisungsoperator.<BR>
     * Besitzt aufgrund der Singleton-Struktur,<BR>
     * keine Aufgabe und ist daher auch private
     * @return
     */
    FlightController& operator =(FlightController const&) = delete;

    /**
      * @brief Standard-Destruktor zum Löschen der FlightController-Instanz.<BR>
      * Aufgrund der Singleton-Struktur private und ohne Aufgabe
      */
    ~FlightController();

public:

    /**
     * @brief Überprüft, ob die Verbindung zu ROS besteht
     * @return Wahrheitswert der angibt, dass die Verbindung besteht
     */
    bool isConnected();

    /**
     * @brief Übergibt Kommandos an ROS zur Steuerung der Drohne
     * @param p_Command Das auszuführende Kommando
     */
    void publishCommand(geometry_msgs::Twist p_Command);

    /**
     * @brief Sendet den Befehl "abheben" an ROS
     */
    void takeOff();

    /**
     * @brief Sendet den Befehl "landen" an ROS
     */
    void land();

    /**
     * @brief Startet den Thread
     */
    void startThread();

    /**
     * @brief Stoppt den Thread
     */
    void stopThread();

    /**
     * @brief Legt die Frequenz fest, wann Kommanos ausgeführt werden sollen
     * @param p_Frequency Neue Frequenz
     */
    void setAutoRepeat(int p_Frequency);

    /**
     * @brief Gibt an, ob das vorherige Kommando ausgeführt werden soll
     * @param p_RunLastCommand Wahrheitswert der angibt,<BR>
     * ob das letzte Kommando ausgeführt werden soll, oder nicht
     */
    void setRunLastCommand(bool p_RunLastCommand);

    /**
     * @brief Legt den Modus fest, wie Kommandos ausgeführt werden sollen
     * @param p_CommandType Kommando-Typ
     * @see Globals::CommandType
     */
    void setCommandType(Globals::CommandType p_CommandType);

    /**
     * @brief Gibt den aktuell verwendeten Kommando-Typ zurück
     * @return Aktueller Kommando-Typ
     */
    Globals::CommandType getCommandType();

    /**
     * @brief Liefert die Singleton-Instanz des FlightControllers
     * @return FlightController-Singleton-Instanz
     */
    static FlightController &getInstance();

};

#endif //__FLIGHT_CONTROLLER_H
