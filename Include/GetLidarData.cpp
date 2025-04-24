#include "GetLidarData.h"

GetLidarData::GetLidarData() {

    for (long int FF = 0; FF < 360000; FF++)
    {
        cosAngleValue[FF] = cos(FF / 1000.0 * PI / 180);
        sinAngleValue[FF] = sin(FF / 1000.0 * PI / 180);
    }

	LidarPerFrameDatePrt_Get = std::make_shared<std::vector<MuchLidarData>>();
}

GetLidarData::~GetLidarData() {

}


void GetLidarData::setPortAndIP(uint16_t mDataPort, uint16_t mDevPort, std::string mDestIP, std::string mLidarIP, std::string mGroupIp)
{
	dataPort = mDataPort;
	devPort = mDevPort;
	computerIP = mDestIP;
    lidarIP = mLidarIP;
    groupIp = mGroupIp;
}


void GetLidarData::setChangeLidarIP(std::string mFixedLidarIP, std::string mLidarIP, std::string mDestIP, uint16_t mDataPort, uint16_t mDevPort)
{
	std::string str = "This version of Lidar does not support 'setChangeLidarIP()'!!!";
	messFunction(str, 0);
    return;
}

bool GetLidarData::sendSecondIP(int switchIP, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'sendSecondIP()'!!!";
    return false;
}

bool GetLidarData::sendSecondCueLight(int switchIP, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'sendSecondCueLight()'!!!";
    return false;
}

void GetLidarData::setCallbackFunction(FunDataPrt *callbackValue) {
    callback = callbackValue;
}

void GetLidarData::LidarStart() {
    isQuit = false;
    std::thread t1(&GetLidarData::LidarRun, this);
    t1.detach();

#ifdef LINUX
    std::thread m_LinuxSockT(&GetLidarData::LinuxSockCreate, this);
    m_LinuxSockT.detach();
#else
    //CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)getWindowSock, NULL, 0, NULL);
    std::thread m_WindowSockT(&GetLidarData::WindowSockCreate, this);
    m_WindowSockT.detach();

#endif

}

void GetLidarData::LidarOfflineDataStar()
{
	isQuit = false;
	std::thread t1(&GetLidarData::LidarRun, this);
	t1.detach();
}

#ifdef LINUX

int setNonBlocking(int sockfd) {
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags == -1) {
        return -1;
    }
    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) == -1) {
        return -1;
    }
    return 0;
}



//Obtain the port number of the device packet
int GetLidarData::LinuxSockCreate()
{
    struct epoll_event events[2];
    int epoll_fd = epoll_create1(0);
    if (epoll_fd == -1)
     {
        std::cerr << "Failed to create epoll file descriptor\n";
	 	std::string str = "Failed to create epoll file descriptor\n";
        messFunction(str, 10000);
        return 1;
     }

    int server_sockfd[2];
    struct sockaddr_in server_addr1, server_addr2;

    // Create two UDP socket sockets
    for (int i = 0; i < 2; ++i) {
        server_sockfd[i] = socket(AF_INET, SOCK_DGRAM, 0);
        if (server_sockfd[i] == -1) {
            std::cerr << "Failed to create socket\n";
			
			std::string str = "Failed to create socket!\n";
        	messFunction(str, 10000);
            return 1;
        }

 		memset(&server_addr1, 0, sizeof(server_addr1));
 	
        server_addr1.sin_family = AF_INET;
        server_addr1.sin_addr.s_addr = INADDR_ANY;
		
		if(i == 0)
		{
			server_addr1.sin_port = htons(dataPort);

            int value = 10 * 1024 * 1024;
            int tmpCode = 0;
            tmpCode = ::setsockopt(server_sockfd[i], SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(value));
		}
		else
		{
			server_addr1.sin_port = htons(devPort);
		}

        ip_mreq multiCast;
        multiCast.imr_interface.s_addr = INADDR_ANY;
        //multiCast.imr_multiaddr.s_addr = inet_addr(groupIp.c_str());
        inet_pton(AF_INET, groupIp.data(), &multiCast.imr_multiaddr.s_addr);

        if (setsockopt(server_sockfd[i], IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &multiCast, sizeof(multiCast)) < 0) 
        {
            std::string str = "Adding multicast groupIP error!\n";
        	messFunction(str, 10000);
            //close(sockfd_);
            //return 1;
        }
         else
        {
            std::cout << "Adding multicast groupIP...OK.\n" ;
        } 

        if (bind(server_sockfd[i], (struct sockaddr*)&server_addr1, sizeof(server_addr1)) == -1) {
            std::cerr << "Failed to bind socket\n";
		 	std::string str = "Failed to bind socket\n";
        	messFunction(str, 10002);
            return 1;
        }

       setNonBlocking(server_sockfd[i]);

        struct epoll_event event;
        //event.events = EPOLLIN | EPOLLET;  // Edge-triggered
        event.events = EPOLLIN; 
        event.data.fd = server_sockfd[i];

        if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, server_sockfd[i], &event) == -1) {
            std::cerr << "Failed to add socket to epoll\n";
			std::string str = "Failed to add socket to epoll\n";
        	messFunction(str, 10002);
            return 1;
    }
    }

    while (true)
	 {
	    if (isQuit)
        {
            std::string str = "Exit to obtain network data!!!\n";
            messFunction(str, 10009);
            break;
        }
        int num_events = epoll_wait(epoll_fd, events, 2, 2000);// 2 seconds
        if (num_events == -1) {
            std::cerr << "Error in epoll_wait\n";
            return 1;
        }
        else if (num_events == 0) {
            std::cout << "Timeout. No events occurred within " << 2000 << " ms\n";
			std::string str = "Timeout!!! 2s\n";
        	messFunction(str, 10004);
            continue;
        }

        for (int i = 0; i < num_events; ++i) {
            if (events[i].events & EPOLLIN) {
                unsigned char buffer[2048];
                struct sockaddr_in client_addr;
                socklen_t addr_len = sizeof(client_addr);
                int bytes_received = recvfrom(events[i].data.fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_addr, &addr_len);
                if (bytes_received == -1) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
						std::string str = "No more data to read \n";
        				messFunction(str, 10004);
                    }
                    else {
                        std::cerr << "Error in recvfrom\n";
						std::string str = "Error in recvfrom \n";
        				messFunction(str, 10003);
                        return 1;
                    }
                }
                else {
                
                u_char data[1212] = { 0 };
                memcpy(data, buffer, bytes_received);
                CollectionDataArrive(data, bytes_received);			//transmit data to class
                }
            }
        }
    }

    close(epoll_fd);
    for (int i = 0; i < 2; ++i) {
        close(server_sockfd[i]);
    }

    return 0;
}

#else
//obtain data packet port number
void GetLidarData::WindowSockCreate()
{
    //create the completion port
    HANDLE completionPort = CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 0);
    if (completionPort == NULL) {
        std::cerr << "CreateIoCompletionPort failed" << std::endl;
        std::string str = "CreateIoCompletionPort failed!\n";
        messFunction(str, 10000);
        return;
    }

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed" << std::endl;
        std::string str = "WSAStartup failed!\n";
        messFunction(str, 10000);
        return;
    }

#pragma region //
    // create the dp socket and the connection completion port
    m_SockData = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_SockData == INVALID_SOCKET) {
        std::cerr << "The m_SockData socket create failed" << std::endl;
        std::string str = "The m_SockData socket create failed!\n";
        messFunction(str, 10000);
        return;
    }

    //define address
    struct sockaddr_in sockAddr;
    sockAddr.sin_family = AF_INET;
    sockAddr.sin_port = htons(dataPort);
    //sockAddr.sin_addr.s_addr = inet_addr(computerIP.c_str());
    inet_pton(AF_INET, computerIP.c_str(), &sockAddr.sin_addr);

    int value = 10 * 1024 * 1024;
    int tmpCode = 0;
    tmpCode = ::setsockopt(m_SockData, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(value));

    ip_mreq multiCast;
    multiCast.imr_interface.S_un.S_addr = INADDR_ANY;		         //IP address of a local network device interface
    inet_pton(AF_INET, groupIp.data(), &multiCast.imr_multiaddr.S_un.S_addr);

   // setsockopt(m_SockData, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&multiCast, sizeof(multiCast));

    if (setsockopt(m_SockData, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&multiCast, sizeof(multiCast)) < 0)
    {
        std::string str = "Adding multicast groupIP error!\n";
        messFunction(str, 10000);
        //close(sockfd_);
        //return 1;
    }
    else
    {
        std::cout << "Adding multicast groupIP...OK.\n";
    }
	
    if (bind(m_SockData, (SOCKADDR*)&sockAddr, sizeof(sockAddr)) == SOCKET_ERROR) {
        std::cerr << "Error binding server socket" << std::endl;
        std::string str = "Bind m_SockData failed!!!\n";
        messFunction(str, 10001);
        return;
    }

#pragma endregion 

#pragma region //
    //create the dp socket and the connection completion port
    m_SockDev = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_SockDev == INVALID_SOCKET) {
        std::cerr << "The m_SockDev socket create failed" << std::endl;
        std::string str = "The m_SockDev socket create failed!\n";
        messFunction(str, 10000);
        return;
    }

    //define address
    sockAddr.sin_family = AF_INET;
    sockAddr.sin_port = htons(devPort);
    //sockAddr.sin_addr.s_addr = inet_addr(computerIP.c_str());
    inet_pton(AF_INET, computerIP.c_str(), &sockAddr.sin_addr);

    multiCast;
    multiCast.imr_interface.S_un.S_addr = INADDR_ANY;		         //IP address of a local network device interface
    inet_pton(AF_INET, groupIp.data(), &multiCast.imr_multiaddr.S_un.S_addr);

    if (setsockopt(m_SockDev, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&multiCast, sizeof(multiCast)) < 0)
    {
        std::string str = "Adding multicast groupIP error!\n";
        messFunction(str, 10000);
        //close(sockfd_);
        //return 1;
    }
    else
    {
        std::cout << "Adding multicast groupIP...OK.\n";
    }
	
    if (bind(m_SockDev, (SOCKADDR*)&sockAddr, sizeof(sockAddr)) == SOCKET_ERROR) {
        std::cerr << "Error binding server socket" << std::endl;
        std::string str = "Bind m_SockDev failed!!!\n";
        messFunction(str, 10011);
        return;
    }

#pragma endregion 


    // connection socket and completion port
    CreateIoCompletionPort((HANDLE)m_SockData, completionPort, (ULONG_PTR)m_SockData, 0);
    CreateIoCompletionPort((HANDLE)m_SockDev, completionPort, (ULONG_PTR)m_SockDev, 0);

    // start asynchronous IO operation
    SocketContext context1 = { m_SockData, {0}, WSAOVERLAPPED(), {1212, context1.buffer} };
    SocketContext context2 = { m_SockDev, {0}, WSAOVERLAPPED(), {1212, context2.buffer} };


    DWORD flags = 0;
    DWORD bytesTransferred = 0;

    int result = WSARecv(m_SockData, &context1.dataBuf, 1, &bytesTransferred, &flags, &context1.overlapped, NULL);
    if (result == SOCKET_ERROR) {
        if (WSAGetLastError() != WSA_IO_PENDING) {
            std::cerr << "m_SockData WSARecvFrom failed" << std::endl;
            std::string str = "The m_SockData failed to obtain data!!!\n";
            messFunction(str, 10003);
            return;
        }
    }


    result = WSARecv(m_SockDev, &context2.dataBuf, 1, &bytesTransferred, &flags, &context2.overlapped, NULL);
    if (result == SOCKET_ERROR) {
        if (WSAGetLastError() != WSA_IO_PENDING) {
            std::cerr << "m_SockDev WSARecvFrom failed" << std::endl;
            std::string str = "The m_SockDev failed to obtain data!!!\n";
            messFunction(str, 10013);
            return;
        }
    }

    while (true)
    {
        if (isQuit)
        {
            std::string str = "Exit to obtain network data!!!\n";
            messFunction(str, 10019);
            break;
        }

        DWORD dwBytes;
        ULONG_PTR ulKey;
        LPOVERLAPPED lpOverlapped;
        // set the timeout time for 1 second
        BOOL bResult = GetQueuedCompletionStatus(completionPort, &dwBytes, &ulKey, &lpOverlapped, 1000);

        if (!bResult) {
            // handling error
            DWORD dwError = GetLastError();
            if (dwError == ERROR_NETNAME_DELETED) {
                // the client disconnects the connection
            }
            else {
                std::cout << "timeOut Error!: " << dwError << std::endl;
                std::string str = " socket recvfrom timeout ! Failed to obtain data!\n";
                messFunction(str, 10004);
            }
        }
        else {
            if (ulKey == (ULONG_PTR)m_SockData) {
                // std::cout << "Socket 1 received " << dwBytes << " bytes of data." << std::endl;

                u_char data[1212] = { 0 };
                memcpy(data, context1.dataBuf.buf, dwBytes);
                CollectionDataArrive(data, dwBytes);			//transmit data to class

                 // launch the next asynchronous IO operation
                WSARecv(m_SockData, &context1.dataBuf, 1, &bytesTransferred, &flags, &context1.overlapped, NULL);
            }

            if (ulKey == (ULONG_PTR)m_SockDev) {
                // std::cout << "Socket 2 received " << dwBytes << " bytes of data." << std::endl;
                u_char data[1212] = { 0 };
                memcpy(data, context2.dataBuf.buf, dwBytes);
                CollectionDataArrive(data, dwBytes);			//transmit data to class

                // launch the next asynchronous IO operation
                WSARecv(m_SockDev, &context2.dataBuf, 1, &bytesTransferred, &flags, &context2.overlapped, NULL);
            }

        }
    }

    // clean up resources
    closesocket(m_SockData);
    closesocket(m_SockDev);
    CloseHandle(completionPort);
    WSACleanup();

}

#endif
std::string GetLidarData::getDataPacketState()
{
    m_mutex.lock();
    std::string mDataInfoStringT = mDataInfoString;
    m_mutex.unlock();

    return mDataInfoStringT;
}

std::string GetLidarData::getDevPacketState()
{
    m_mutex.lock();
    std::string mDevInfoStringT = mDataInfoString;
    m_mutex.unlock();
 
    return mDevInfoStringT;
}

bool GetLidarData::getLidarParamState(LidarStateParam& mLidarStateParam, std::string& InfoString)
{
    if (true == islidarDevCome)
    {
        m_mutex.lock();
        //Save the device packet
        unsigned char pktdata[1206];												//modify configuration packet
        memcpy(pktdata, dataDev, 1206);
        m_mutex.unlock();

        float mMotorSpeed = pktdata[8] * 256 + pktdata[9];
        mLidarStateParam.MotorSpeed = mMotorSpeed;

        //Lidar IP
        std::string ip_value = 
            std::to_string(pktdata[10]) + "." +
            std::to_string(pktdata[11]) + "." +
            std::to_string(pktdata[12]) + "." +
            std::to_string(pktdata[13]);
        mLidarStateParam.LidarIP = ip_value;

        //Lidar destination IP（computer IP）
        ip_value =
            std::to_string(pktdata[14]) + "." +
            std::to_string(pktdata[15]) + "." +
            std::to_string(pktdata[16]) + "." +
            std::to_string(pktdata[17]);
        mLidarStateParam.ComputerIP = ip_value;

        //Lidar NTP IP（NTP IP）
        ip_value =
            std::to_string(pktdata[28]) + "." +
            std::to_string(pktdata[29]) + "." +
            std::to_string(pktdata[30]) + "." +
            std::to_string(pktdata[31]);
        mLidarStateParam.NtpIP = ip_value;

        //Lidar Gateway IP（Gateway IP）
        ip_value =
            std::to_string(pktdata[32]) + "." +
            std::to_string(pktdata[33]) + "." +
            std::to_string(pktdata[34]) + "." +
            std::to_string(pktdata[35]);
        mLidarStateParam.GatewayIP = ip_value;

        //Lidar Subnet Mask IP（Subnet Mask IP）
        ip_value =
            std::to_string(pktdata[36]) + "." +
            std::to_string(pktdata[37]) + "." +
            std::to_string(pktdata[38]) + "." +
            std::to_string(pktdata[39]);
        mLidarStateParam.SubnetMaskIP = ip_value;

        ip_value =
            IntToHex(pktdata[18]) + "-" +
            IntToHex(pktdata[19]) + "-" +
            IntToHex(pktdata[20]) + "-" +
            IntToHex(pktdata[21]) + "-" +
            IntToHex(pktdata[22]) + "-" +
            IntToHex(pktdata[23]);
        mLidarStateParam.MacAddress = ip_value;

        mLidarStateParam.DataPort = pktdata[24] * 256 + pktdata[25];
        mLidarStateParam.DevPort  = pktdata[26] * 256 + pktdata[27];

        return true;
    }
    else
    {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

void GetLidarData::LidarStop() {
    isQuit = true;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


void GetLidarData::sendLidarData() {

    if (m_DistanceIsNotZero < 20)
    {
        messFunction("Data error!!! All lidar distanceValue are 0!", 10032);
    }
    else
    {
        m_mutex.lock();
        isSuccessfulFlag = true;
        mDataInfoString = "Obtaining data successfully!";
        m_mutex.unlock();
    }
    m_DistanceIsNotZero = 0;

	if (callback) {
		LidarPerFrameDatePrt_Send = LidarPerFrameDatePrt_Get;
		(*callback)(LidarPerFrameDatePrt_Send, isSuccessfulFlag, mDataInfoString);
	}
    m_mutex.lock();
	LidarPerFrameDatePer = LidarPerFrameDatePrt_Get;
	LidarPerFrameDatePrt_Get.reset(new std::vector<MuchLidarData>); 
	isFrameOK = true;
    m_mutex.unlock();
}

void GetLidarData::CollectionDataArrive(void *pData, uint16_t len) {
    if (len >= 1206) {
        unsigned char *dataV = new unsigned char[1212];
        memset(dataV, 0, 1212);
        memcpy(dataV, pData, len);
        m_mutex.lock();
        allDataValue.push(dataV);
        m_mutex.unlock();

        if ((dataV[0] == 0x00 || dataV[0] == 0xa5) && dataV[1] == 0xff && dataV[2] == 0x00 && dataV[3] == 0x5a) 
        {
            m_mutex.lock();
            memcpy(dataDev, pData, 1206);
            ip_sa = std::to_string(dataDev[10]) + "." +
                    std::to_string(dataDev[11]) + "." +
                    std::to_string(dataDev[12]) + "." +
                    std::to_string(dataDev[13]);

            if(setIndex > 0) std::cout << "Current index parameters:" << setIndex << "	The read-back value is: " << ": " << (setIndex < 0 ? 0 : int(dataV[setIndex])) << std::endl;
            islidarDevCome = true;
            m_mutex.unlock();
        }
    }
}

int  GetLidarData::NegativeToPositive(float value)
{
    int valueT = value * 1000;
    if (valueT >= 0)
    {
        return (valueT > 360000 ? valueT % 360000 : valueT);
    }
    else
    {
        return (valueT < -360000 ? (valueT % -360000) + 360000 : valueT + 360000);
    }
}

std::string GetLidarData::ucharToBinaryStr(const unsigned char value)
{
    std::string binaryStr;
    for (int i = 7; i >= 0; i--) {
        binaryStr += (value & (1 << i)) ? '1' : '0';
    }
    return binaryStr;
}

void GetLidarData::clearQueue(std::queue<unsigned char *> &m_queue) {
    std::queue<unsigned char *> empty;
    swap(empty, m_queue);
}

bool GetLidarData::getLidarPerFrameDate(std::shared_ptr<std::vector<MuchLidarData>>& preFrameData, std::string& Info)
{
    m_mutex.lock();
    isFrameOK = false;
    preFrameData = std::move(LidarPerFrameDatePer);
    Info = mDataInfoString;
    m_mutex.unlock();
    return isSuccessfulFlag;

}

#pragma region //Set radar parameters to send device packet 

//set the rotate speed
bool GetLidarData::setLidarRotateSpeed(int SpeedValue, std::string& InfoString) {

	m_SpeedValue = SpeedValue;
	
	if (setLidarParam()) {
	    //set the rotate speed
		Rest_UCWP_buff[8] = SpeedValue / 256;	
		Rest_UCWP_buff[9] = SpeedValue % 256;
	} else {
        InfoString = "Equipment package is not update!!!";
	    return false;
	}
	startSleepThread();
	return true;
}
//Turn on the speed judgment after a period of time
void GetLidarData::sleepTime()
{
	is_speedFlag = false;
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	is_speedFlag = true;
}

//Start the speed judgement thread
void GetLidarData::startSleepThread()
{
	std::thread t1_Start(&GetLidarData::sleepTime, this);
	t1_Start.detach();
}

//set lidar IP
bool GetLidarData::setLidarIP(std::string LidarIPValue, std::string& InfoString) {
    std::regex ipv4(
            "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
    if (!regex_match(LidarIPValue, ipv4)) {
        InfoString = "The IP format entered is incorrect, please check the input parameters";
        return false;
    }
    if (setLidarParam()) {
        //set lidar IP
        std::string::size_type defailtIP_pos;
        std::vector<std::string> IP_result;
        IP_result.clear();
        LidarIPValue = LidarIPValue + ".";                                                     //Easily obtain the last piece of data
        for (size_t i = 0; i < LidarIPValue.size(); i++)                                     //cut out defaultIP lineedit
        {
            defailtIP_pos = LidarIPValue.find(".", i);
            if (defailtIP_pos < LidarIPValue.size()) {
                std::string s = LidarIPValue.substr(i, defailtIP_pos - i);
                IP_result.emplace_back(std::move(s));
                i = defailtIP_pos;// +pattern.size() - 1;
            }
        }

        if (IP_result.size() < 4) {
            InfoString = "Please enter the full Lidar IP address!!!Failed to set the Lidar IP address!!!";
            return false;
        } else if (IP_result.size() == 4 && IP_result[3] == "") {
            InfoString =  "Please enter the full Lidar IP address!!!Failed to set the Lidar IP address!!!";
            return false;
        }
    
        if (!checkDefaultIP(IP_result, InfoString)) {
            InfoString =  "Failed to set the Lidar IP address!!!";
            return false;
        }
    
        Rest_UCWP_buff[10] = atoi(IP_result[0].c_str());
        Rest_UCWP_buff[11] = atoi(IP_result[1].c_str());
        Rest_UCWP_buff[12] = atoi(IP_result[2].c_str());
        Rest_UCWP_buff[13] = atoi(IP_result[3].c_str());
        InfoString = "Successfully set!";
        return true;
    } else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

//Check whether the lidar IP settings are in compliance with the requireement
bool GetLidarData::checkDefaultIP(std::vector<std::string> m_DefaultIP, std::string& InfoString) {
    int HeadDefaultIPValue = stoi(m_DefaultIP[0]);
    int endDefaultIPValue = stoi(m_DefaultIP[3]);
    if (HeadDefaultIPValue == 0 || HeadDefaultIPValue == 127 ||
        (HeadDefaultIPValue >= 224 && HeadDefaultIPValue <= 255)
            ) {
        std::string str =
                "The Lidar IP cannot be set to " + std::to_string(HeadDefaultIPValue) + std::string(".x.x.x!!!");
        InfoString = str;
        messFunction(str, 0);
        return false;
    } else if (endDefaultIPValue == 255) {
        std::string str = "The Lidar IP cannot be set to broadcast(x.x.x.255)!!!";
        InfoString = str;
        messFunction(str, 0);
        return false;
    }
    return true;
}

//Check whether the destination IP settings are in compliance with the requireement
bool GetLidarData::checkDestIP(std::vector<std::string> m_DestIP, std::string& InfoString) {
    int HeadDefaultIPValue = stoi(m_DestIP[0]);
    int endDefaultIPValue = stoi(m_DestIP[3]);
    if (HeadDefaultIPValue == 0 || HeadDefaultIPValue == 127 ||
        (HeadDefaultIPValue >= 240 && HeadDefaultIPValue <= 255)
            ) {
        std::string str =
                "The Dest IP cannot be set to " + std::to_string(HeadDefaultIPValue) + std::string(".x.x.x!!!");
        InfoString = str;
        messFunction(str, 0);
        return false;
    }
    return true;
}

//set computer IP
bool GetLidarData::setComputerIP(std::string ComputerIPValue, std::string& InfoString) {
    std::regex ipv4(
            "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
    if (!regex_match(ComputerIPValue, ipv4)) {
        InfoString = "The IP format entered is incorrect, please check the input parameters";
        return false;
    }

    if (setLidarParam()) {
        //set computer IP
        std::vector<std::string> IP_result;
        IP_result.clear();
        std::string::size_type DestIP_pos;
        ComputerIPValue = ComputerIPValue + ".";
        for (size_t i = 0; i < ComputerIPValue.size(); i++)                                     //cut out of destIP lineedit 
        {
            DestIP_pos = ComputerIPValue.find(".", i);
            if (DestIP_pos < ComputerIPValue.size()) {
                std::string s = ComputerIPValue.substr(i, DestIP_pos - i);
                IP_result.emplace_back(std::move(s));
                i = DestIP_pos;
            }
        }
    
        if (IP_result.size() < 4) {
            InfoString = "Please enter the full Dest IP address!!! Failed to set the Dest(Computer) IP address!!!";
            return false;
    
        } else if (IP_result.size() == 4 && IP_result[3] == "") {
            InfoString = "Please enter the full Dest IP address!!!Failed to set the Dest(Computer) IP address!!!";
            return false;
        }
        if (!checkDestIP(IP_result, InfoString)) {
            InfoString = "Failed to set the computer IP address!!!";
            return false;
        }
        Rest_UCWP_buff[14] = atoi(IP_result[0].c_str());
        Rest_UCWP_buff[15] = atoi(IP_result[1].c_str());
        Rest_UCWP_buff[16] = atoi(IP_result[2].c_str());
        Rest_UCWP_buff[17] = atoi(IP_result[3].c_str());
    
        return true;
    } else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

bool GetLidarData::setNTP_IP(std::string IPString, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'setNTP_IP()'!!!";
    return false;
}

bool GetLidarData::setGatewayIP(std::string IPString, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'setGatewayIP()'!!!";
    return false;
}

bool GetLidarData::setSubnetMaskIP(std::string IPString, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'setSubnetMaskIP()'!!!";
    return false;
}

//set data packet port
bool GetLidarData::setDataPort(int DataPort, std::string& InfoString) {
    if (setLidarParam()) {
        //check port
        int devPort = Rest_UCWP_buff[26] * 256 + Rest_UCWP_buff[27];
        if (DataPort < 1025 || DataPort > 65535 || DataPort == devPort) {
            InfoString = "DataPort range 1025-65535 and DataPort and devport cannot be equal, please check the input parameters";
            return false;
        } else {
            //set data packet port
            Rest_UCWP_buff[24] = DataPort / 256;
            Rest_UCWP_buff[25] = DataPort % 256;
            return true;
        }

    } else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

//set device packet port
bool GetLidarData::setDevPort(int DevPort, std::string& InfoString) {
    if (setLidarParam()) {
        //check port
        int dataPort = Rest_UCWP_buff[24] * 256 + Rest_UCWP_buff[25];
        if (DevPort < 1025 || DevPort > 65535 || DevPort == dataPort) {
            InfoString = "DataPort range 1025-65535 and DataPort and devport cannot be equal, please check the input parameters";
            return false;
        } else {
            //set device packet port
            Rest_UCWP_buff[26] = DevPort / 256;
            Rest_UCWP_buff[27] = DevPort % 256;
            return true;
        }
    } else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

bool GetLidarData::setLidarRotateState(int RotateState, std::string& InfoString) {
    InfoString = "This version of Lidar does not support 'setLidarRotateState()'!!!";
	return false;
}

bool GetLidarData::setLidarSoureSelection(int StateValue, std::string& InfoString) {
    InfoString = "This version of Lidar does not support 'setLidarSoureSelection()'!!!";
    return false;
}

bool GetLidarData::setLidarWorkState(int LidarState, std::string& InfoString) {
    InfoString = "This version of Lidar does not support 'setLidarWorkState()'!!!";
    return false;
}

bool GetLidarData::setFrameRateMode(int StateValue, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'setFrameRateMode()'!!!";
    return false;
}

bool GetLidarData::setPhaseLockedSwitch(int StateValue, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'setPhaseLockedSwitch()'!!!";
    return false;
}

bool GetLidarData::setLidarIndexParamValue(int ControlValue, int ChangeValue, std::string& InfoString)
{
    if (setLidarParam()) 
    {
        Rest_UCWP_buff[ControlValue] = ChangeValue;
        std::cout << "setLidarIndexParamValue() : ControlValue = " << ControlValue << " : ChangeValue = " << ChangeValue << std::endl;
        return true;
    }
    else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}


bool GetLidarData::setLidarParam() {
    if (isSendUDP == false) {
        return true;
    }

    if (true == islidarDevCome && true == isSendUDP) {
        islidarDevCome = false;
        isSendUDP = false;
        m_mutex.lock();
        //Save the device packet before send the configuration packet
        memcpy(Rest_UCWP_buff, dataDev, 1206);
        m_mutex.unlock();

        for (int i = 52; i < 60; i++) {
            Rest_UCWP_buff[i] = 0x00;
        }
    
        for (int i = 160; i < 168; i++) {
            Rest_UCWP_buff[i] = 0x00;
        }
    
        Rest_UCWP_buff[0] = 0xAA;                           //merge UCWP with ACWP, the UCWP identification header is the first 8 bytes
        Rest_UCWP_buff[1] = 0x00;
        Rest_UCWP_buff[2] = 0xFF;
        Rest_UCWP_buff[3] = 0x11;
        Rest_UCWP_buff[4] = 0x22;
        Rest_UCWP_buff[5] = 0x22;
        Rest_UCWP_buff[6] = 0xAA;
        Rest_UCWP_buff[7] = 0xAA;
        return true;
    } else {
        std::string str = "Equipment package is not update!!!";
        messFunction(str, 0);
        return false;
    }

}

bool GetLidarData::sendPacketToLidar(unsigned char *packet, const char *ip_data, u_short portNum) {
#ifdef __linux__
    struct sockaddr_in addrSrv{};
    //create socket UDP
    int socketid = socket(2, 2, 0);
#define SOCKET_ERROR -1
#else
    //initialize socket 
    WORD  request;
    WSADATA  ws;
    request = MAKEWORD(1, 1);
    int err = WSAStartup(request, &ws);
    if (err != 0)
    {
        return false;
    }
    if (LOBYTE(ws.wVersion) != 1 || HIBYTE(ws.wVersion) != 1)
    {
        WSACleanup();
        return false;
    }
    SOCKADDR_IN addrSrv;
    //create socket UDP
    SOCKET socketid = socket(2, 2, 0);
#endif

    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(portNum);
    inet_pton(AF_INET, ip_data, &addrSrv.sin_addr);
    
    int sd = sendto(socketid, (const char *) packet, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
    
    if (sd != SOCKET_ERROR) {
        printf("send successfully,send:%dchars\n", sd);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
#ifdef __linux__
        (void) ::close(socketid);
#else
        closesocket(socketid);
#endif
    } else {
        printf("Failure to send\n");
        return false;
    }

    isSendUDP = true;
    return true;
}

bool GetLidarData::sendPackUDP() {
    return sendPacketToLidar(Rest_UCWP_buff, ip_sa.c_str(), 2368);
}

void GetLidarData::messFunction(std::string strValue, int gValue) {
    std::cout << "Code = " << gValue << " : " << strValue.c_str() << std::endl;

    m_mutex.lock();
    if ((10000 <= gValue  &&  gValue <= 10009) || (10030 <= gValue && gValue <= 10039))
    {
        isSuccessfulFlag = false;
        mDataInfoString = strValue;   
    }

    if ((10010 <= gValue && gValue <= 10019))
    {
        mDevInfoString = strValue;
    }
    m_mutex.unlock();

}

#pragma endregion


#pragma region//lidar filter display Settings

bool GetLidarData::setLidarFilterdisplay(LidaFilterParamDisplay mLidaFilterParamDisplay)
{
    mLidaFilterParamDisplayValue = mLidaFilterParamDisplay;
    return true;
}

bool GetLidarData::isPointFilter(const MuchLidarData mPoint)
{
    //Channel filtering
    if (mPoint.ID < mLidaFilterParamDisplayValue.mChannelVector.size())
        if (0 == mLidaFilterParamDisplayValue.mChannelVector[mPoint.ID])
            return false;

    //Horizontal Angle filtering
    if (mLidaFilterParamDisplayValue.mMin_HanleValue <= mLidaFilterParamDisplayValue.mMax_HanleValue)
    {
        if (!(mLidaFilterParamDisplayValue.mMin_HanleValue <= mPoint.H_angle && mPoint.H_angle <= mLidaFilterParamDisplayValue.mMax_HanleValue))
            return false;

    }
    else
    {
        if (mLidaFilterParamDisplayValue.mMax_HanleValue < mPoint.H_angle && mPoint.H_angle < mLidaFilterParamDisplayValue.mMin_HanleValue)
            return false;
    }

    //Vertical Angle filtering
    if (!(mLidaFilterParamDisplayValue.mMin_VanleValue <= mPoint.V_angle && mPoint.V_angle <= mLidaFilterParamDisplayValue.mMax_VanleValue))
        return false;


    //Distance filtering
    if (!(mLidaFilterParamDisplayValue.mMin_Distance <= mPoint.Distance && mPoint.Distance <= mLidaFilterParamDisplayValue.mMax_Distance))
        return false;
 
    //Intensity filtering
    if (!(mLidaFilterParamDisplayValue.mMin_Intensity <= mPoint.Intensity && mPoint.Intensity <= mLidaFilterParamDisplayValue.mMax_Intensity))
        return false;

    return true;
}
#pragma endregion