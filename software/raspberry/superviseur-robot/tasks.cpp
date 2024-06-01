/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// D�claration des priorit�s des taches
#define PRIORITY_TSERVER 45
#define PRIORITY_TOPENCOMROBOT 40
#define PRIORITY_TMOVE 32
#define PRIORITY_TSENDTOMON 35
#define PRIORITY_TRECEIVEFROMMON 33
#define PRIORITY_TSTARTROBOT 30
#define PRIORITY_TCAMERA 30

#define PRIORITY_TBATTERY 10
#define PRIORITY_TWATCHDOG 41
#define PRIORITY_ARENA 31




/**
 * @brief Initialisation des structures de l'application (t�ches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;//status de routeur des fonctions 
    int err; //code d'erreur

    
    cam = new Camera(sm, 5);// intialisation de camera

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) { // creation de mutex pour moniteur 
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) { // creation de mutex pour roboot
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) { // creation de mutex pour l'etat du robot
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) { // creation de mutex pour les movements
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_counter, NULL)) { // creation du mutex pour le compteur
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cam, NULL)) { // creation du mutex pour la camera
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_batteryRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_watchdog, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_Arena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_ArenaOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
    cerr << "Error task create: " << strerror(-err) << endl << flush;
    exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_watchdog, "th_watchdog", 0, PRIORITY_TWATCHDOG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cam, "th_cam", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_Arena, "th_Arena", 0, PRIORITY_ARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    } 
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief D�marrage des t�ches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_watchdog, (void(*)(void*)) & Tasks::WatchdogTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cam, (void(*)(void*)) & Tasks::CameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_Arena, (void(*)(void*)) & Tasks::ArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arr�t des t�ches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            //delete(msgRcv);
            //exit(-1);
            MonitorError(msgRcv);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if(msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD) || msgRcv->CompareID( MESSAGE_ROBOT_START_WITH_WD)) {
            watchdog_id = msgRcv->GetID();
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){
            rt_sem_v(&sem_batteryRobot);
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)){
            cout << "cam open received" << endl;
            OpenCameraTask(arg);
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
            CloseCameraTask(arg);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            rt_sem_v(&sem_Arena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
            rt_sem_v(&sem_ArenaOk);
            arena_ok = 1 ; 
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            rt_sem_v(&sem_ArenaOk);
            arena_ok = 0 ; 
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){  
            robot_position = 1;
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){ 
            robot_position = 0;  
        } 
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // synchronisation de la barriere
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE); // attend le signal pour demmarer le robot

        if(watchdog_id==MESSAGE_ROBOT_START_WITHOUT_WD){
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); // aquisition du mutex du robot
            msgSend = robot.Write(robot.StartWithoutWD()); // demarre le robot sans watchdog
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
        } else {
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); // acquisition du mutex du robot 
            msgSend = robot.Write(robot.StartWithWD());// demarre le robot avec watchdog
            Counter(msgSend);
            rt_mutex_release(&mutex_robot); 
            rt_sem_broadcast(&sem_watchdog);// signale de recherche le watchdog 
            cout << msgSend->GetID();
            cout << ")" << endl;
        }

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // envoie du message au moniteur 

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) { // si le message est un ACK
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);// acquisition du mutex del etat du robot
            robotStarted = 1;// indique aue le robot a demarre
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    Message * answer;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronisation de barriere ( attend que toute les taches soient demarres )
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000); // definit la tache comme periodique avec une periode de 100ms

    while (1) {
        rt_task_wait_period(NULL);// attend la prochaine periode
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE); // acquesition du mutex de l etat du robot
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) { // si le robot a demarre
            rt_mutex_acquire(&mutex_move, TM_INFINITE); // acquisition du mutex des movements
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); // acquisition du mutex du robot
            answer = robot.Write(new Message((MessageID)cpMove)); // envoie le mouvement au robot
            Counter(answer);
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * @brief Thread handling the battery level of the robot.
 */
void Tasks::BatteryTask(void *arg){
    int rs;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronsation de barriere
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000); // definit la tache comme periodique avec une periode de /500ms
    rt_sem_p(&sem_batteryRobot, TM_INFINITE); // attend le signal pour verifier la batterie attendre que le semaphore soit disponible avant de verifier le niveau de batterie

    while (1){
        rt_task_wait_period(NULL); // attend la prochaine periode
        cout << "Periodic battery update" << endl << flush;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE); // acquisiton du mutex de l etat du robot
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        Message * cpLevelBat;
        if (rs == 1) { // si le robot a demarre
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); // acquisition du mutex du robot
            cpLevelBat = robot.GetBattery();// obtient le niveau de batterie du robot
            Message * msg = robot.Write(cpLevelBat);// envoie le niveau de batterie au robot
            Counter(msg);
            rt_mutex_release(&mutex_robot);
            WriteInQueue(&q_messageToMon, msg); // envoie le message au moniteur
        }
        //rt_mutex_release(&mutex_robotStarted);
        cout << endl << flush;
    }
}

void Tasks::WatchdogTask(void *arg) {
    int rs;
    Message * msg;
    Message * answer;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
   
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_watchdog, TM_INFINITE);// attend le signal pour recharge de watchdog
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);  // definit la tache comme periodique avec une periode de 100 ms
    while (1){
        rt_task_wait_period(NULL); // attend la prochaine periode
        cout << "Periodic Watchdog update" << endl << flush; 
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE); // acquisition du mutex de l etat du robot
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1){ // si le robot a demarre
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); // acquisition du mutex du robot
            msg = robot.ReloadWD(); // recharge le watchdog
            answer = robot.Write(msg); // envoie le recharge au robot
            Counter(answer);
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}


// gestion des erreurs de connexion avec le moniteur
void Tasks::Counter(Message * msg) {
    if ((msg->GetID() == MESSAGE_ANSWER_COM_ERROR) || (msg->GetID() == MESSAGE_ANSWER_ROBOT_ERROR) || (msg->GetID() == MESSAGE_ANSWER_ROBOT_TIMEOUT) ){
        counter = counter +1; // incremente le compteur en cas d'erreur
        cout << " Counter + 1 " << __PRETTY_FUNCTION__ << endl << flush;
    } else {
        counter = 0; // renitialise le compteur si pas d erreur
    }
    if (counter >3) { // si plus de 3 erreurs consecutives
        msg = new Message(MESSAGE_MONITOR_LOST);
        cout << "!!!! Connection lost !!!!" << __PRETTY_FUNCTION__ << endl << flush;
        //fermer la communication avec le robot
        cout << "Close Communication" << __PRETTY_FUNCTION__ << endl << flush;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0 ;
        rt_mutex_release(&mutex_robotStarted);
        
        robot.Close();
        //remettre dans �tat initial
        rt_sem_v(&sem_openComRobot);
    }
}

void Tasks::OpenCameraTask(void *arg) {
    bool state;
    Message * msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    cout << "opening the camera" << endl <<flush;
    
    rt_mutex_acquire(&mutex_cam, TM_INFINITE); // acquisition du mutex de la camera
    if(!cam->Open()) { // si l overture echoue 
        msg = new Message(MESSAGE_ANSWER_NACK); // pour message de non ouverture
        cout << "cam not opened " << endl;
    }
    else { // si l ouverture reussit
        msg=new Message(MESSAGE_ANSWER_ACK);
        cout << "cam opened" << endl;
        camOpen = 1; // indique que lq camera est ouverte
        rt_sem_v(&sem_cam); // signale que la camera est ouverte 
        rt_sem_broadcast(&sem_cam);
    }
    rt_mutex_release(&mutex_cam);
    WriteInQueue(&q_messageToMon,msg);    // envoie du message au moniteur
    cout << endl << flush;
}

void Tasks::CloseCameraTask(void *arg) {
    Message * msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    cout << "closing the camera" << endl <<flush;
    rt_mutex_acquire(&mutex_cam, TM_INFINITE);// acquisition du mutex de la camera
    camOpen = 0; // indique que la camera est ferme 
    cam->Close(); // ferme la camera 
    msg = new Message(MESSAGE_ANSWER_ACK);
    rt_mutex_release(&mutex_cam);
    WriteInQueue(&q_messageToMon,msg); // envoie le message au moniteur 
    cout << "camera closed" << endl <<flush;
}

void Tasks::CameraTask(void *arg) {
    rt_task_set_periodic(NULL, TM_NOW, 100000000); // definit une periode de 100ms
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE); 
    
    rt_sem_p(&sem_cam, TM_INFINITE); // attend que la camera s ouvre
    
    while (1) {
        rt_task_wait_period(NULL); // attend la prochaine periode
        rt_mutex_acquire(&mutex_cam, TM_INFINITE); // acquisition du mutex de camera
        if(camOpen){  // si la cemera est ouverte 
            Img * img = new Img(cam->Grab()); // capture d une image
            if (arena_ok){ // si l arene est valide
                img -> DrawArena(arena); // dessine l arenne sur l image
                MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
                WriteInQueue(&q_messageToMon, msgImg); // envoie l image avec l arene au moniteur
            }
            if (robot_position==1){ // si le calcul de la position du robot active
                PositionCalc(img); // calcule la position du robot
            }
            
            MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
            WriteInQueue(&q_messageToMon, msgImg); // envoie l image au moniteur
        }
        rt_mutex_release(&mutex_cam);
    }
}   

void Tasks::ArenaTask(void *arg) {
    rt_task_set_periodic(NULL, TM_NOW, 100000000); // definit une periode de 100ms
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
        rt_task_wait_period(NULL); // attend la prochaine periode 
        cout << " Arena Task " << __PRETTY_FUNCTION__ << endl << flush;
        rt_sem_p(&sem_Arena, TM_INFINITE); // acquisition du semaphore e l arene
        rt_mutex_acquire(&mutex_cam, TM_INFINITE); // acquisition du mutex de la camera
        Img * img = new Img(cam->Grab()); // capture d une image
        arena = img -> SearchArena(); // recherche l arene dans l image
        cout << "ENDDD " << __PRETTY_FUNCTION__ << endl << flush;
        if (arena.IsEmpty()) {// si l image est vide 
            cout << "Empty arena" << __PRETTY_FUNCTION__ << endl << flush;
        } else { // si l arene est detecter
            img -> DrawArena(arena); // dessine l arene sur l image
            MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
            WriteInQueue(&q_messageToMon, msgImg); // envoie de l image avec l arene au moniteur
            cout << "Arena validated" << __PRETTY_FUNCTION__ << endl << flush;
            rt_sem_p(&sem_ArenaOk, TM_INFINITE);// attend la validation de l arene
        }
        rt_mutex_release(&mutex_cam); // libere le mutex de la camera
    }
}

void Tasks::PositionCalc(Img * img) {
    std::list<Position> robotPosition;

    cout << "Search robot" << endl;
    robotPosition =  img->SearchRobot(arena); // recherche les robots dans l image
    if(!robotPosition.empty()){ // si des robots sont dedtectes
        cout << "Draw robots" << endl;
        img->DrawAllRobots(robotPosition); // dessine les positions des robots sur l image 
        for (auto position : robotPosition){
            WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION,position)); // envoie les positions au moniteur 
        }
    } else { // si aucun robot n est detecte
        cout << "Nothing Detected" << endl;
        Position pos = Position();
        pos.center = cv::Point2f(-1.0,-1.0);
        pos.direction = cv::Point2f(-1.0,-1.0);

        WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION,pos)); 
        
    }    
}

void Tasks::MonitorError(Message * msgReceived) {
    if (msgReceived->GetID() == MESSAGE_MONITOR_LOST){
        cout << " Communication Lost Handling " << __PRETTY_FUNCTION__ << endl << flush;
        delete(msgReceived); // supprime lemessage recu 
        robot.Stop(); // arrete le robot
        void * arg;
        CloseCameraTask(arg); // ferme la camera

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE); // acquisition du mutex de l etat du robot
        robotStarted = 0 ; // indique aue le robot est arrete
        rt_mutex_release(&mutex_robotStarted);
        
        robot.Close(); // ferme la communication    vec le robot 

        monitor.AcceptClient(); // attend  un nouveau client pour le moniteur 
        
    } 
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}