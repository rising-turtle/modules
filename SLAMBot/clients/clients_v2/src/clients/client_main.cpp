#include <iostream>
#include <string.h>
#include <stdio.h>
#include "socket_client.h"
#include "sick_client.h"
#include "odo_client.h"
#include "xtion_sick_odo_client.h"
#include "xtion_client.h"

#include "config_file.h"
#include <fstream>

#include "OnlineSLAMClient.h"

#ifdef WIN32
	#include <windows.h>
#else
	#include <pthread.h>
#endif
// #include "xtion_client.h"

using namespace std;

void test_sick_client(int argc, char* argv[]);
void test_config(int argc, char* argv[]);

int main(int argc, char* argv[])
{
    // test_socket_client();
    test_sick_client(argc, argv);
	// test_config(argc, argv);
    return 0; 
}

void test_config(int argc, char* argv[])
{
	string inputf("config.txt"); 
	if(argc == 2)
	{
		inputf = string(argv[1]);
	}
	ifstream inf(inputf.c_str()); 
	bool b_use_xtion_only = true;
	int xtion_port ;
	int sick_port = 9015;
	int odo_port = 9016;
	string server_ip_addr;
	int send_step_ = 4; // send image rate = 1./send_step
	if(!inf.is_open())  // using cmd line
	{
		cout<<inputf<<" not exist, parse cmd line!"; 
		if(argc < 4) 
		{
			cout<<"args should : bool(xtion only) int(port) string(ip) "<<endl;
			return ;
		}else
		{
			b_use_xtion_only = (atoi(argv[1]) == 0)? false: true; 
			xtion_port = atoi(argv[2]);
			server_ip_addr = string(argv[3]);
		}
	}else
	{
		cout<<inputf<<" exist, set params from config file!"<<endl;
		CConfigFile args(inf); 
		args.getParam("xtion_only", b_use_xtion_only, true); 
		args.getParam("server_ip", server_ip_addr, string("192.168.1.112"));
		args.getParam("xtion_port", xtion_port, 9012); 
		args.getParam("sick_port", sick_port, 9015); 
		args.getParam("odo_port", odo_port, 9016);
		args.getParam("send_step", send_step_, 4);
	}
	
	cout<<"xtion_only : "<<(b_use_xtion_only?"TRUE":"FALSE")<<endl;
	cout<<"server_ip : "<<server_ip_addr<<endl;
	cout<<"xtion_port: "<<xtion_port<<endl;
	cout<<"sick_port: "<<sick_port<<endl;
	cout<<"odo_port: "<<odo_port<<endl;
	cout<<"send_step: "<<send_step_<<endl;
}

// args: 1 bool(use xtion only)
//		 2 int (port)
//	     3 string (ip)

void test_sick_client(int argc, char* argv[])
{
    // CXtionClient client; 
	// CSocketClient* client = new CODOClient;
	// CSocketClient* client = new CSickClient;
	// CSocketClient * client = new CSocketClient;
	// CSocketClient * client = new CSickOdoClient;
	
	string inputf("config.txt"); 
	if(argc == 2)
	{
		inputf = string(argv[1]);
	}
	ifstream inf(inputf.c_str()); 
	bool b_use_xtion_only = true;
	int xtion_port ;
	int sick_port = 9015;
	int odo_port = 9016;
	string server_ip_addr;
	int send_step_ = 4; // send image rate = 1./send_step
	if(!inf.is_open())  // using cmd line
	{
		cout<<inputf<<" not exist, parse cmd line!"; 
		if(argc < 4) 
		{
			cout<<"args should : bool(xtion only) int(port) string(ip) "<<endl;
			return ;
		}else
		{
			b_use_xtion_only = (atoi(argv[1]) == 0)?false: true; 
			xtion_port = atoi(argv[2]);
			server_ip_addr = string(argv[3]);
		}
	}else
	{
		cout<<inputf<<" exist, set params from config file!"<<endl;
		CConfigFile args(inf); 
		args.getParam("xtion_only", b_use_xtion_only, true); 
		args.getParam("server_ip", server_ip_addr, string("192.168.1.112"));
		args.getParam("xtion_port", xtion_port, 9012); 
		args.getParam("sick_port", sick_port, 9015); 
		args.getParam("odo_port", odo_port, 9016);
		args.getParam("send_step", send_step_, 4);
	}

	cout<<"xtion_only : "<<(b_use_xtion_only?"TRUE":"FALSE")<<endl;
	cout<<"server_ip : "<<server_ip_addr<<endl;
	cout<<"xtion_port: "<<xtion_port<<endl;
	cout<<"sick_port: "<<sick_port<<endl;
	cout<<"odo_port: "<<odo_port<<endl;
	cout<<"send_step: "<<send_step_<<endl;

	CSocketClient * client; 
	if(b_use_xtion_only)
	{
		client = new CXtionClient;
		client->connect(server_ip_addr.c_str(), xtion_port);
		// client->connect("192.168.0.14", 9013);
	}else
	{
		client = new CXSickOdoClient;
		client->connect(server_ip_addr.c_str(), xtion_port);
		((CXSickOdoClient*)client)->connectBoth(server_ip_addr.c_str(), sick_port, odo_port);
		// ((CXSickOdoClient*)client)->connectBoth("192.168.1.112", 9015, 9016);
	}
	((CXtionClient*)client)->setXtionUploadRate(send_step_);

	// client->connect("192.168.1.112", 9012); // laser 9012 
	Sleep(100);
	cout<<"after connect!"<<endl;
	//. CSocketClient* client2 = new CODOClient;
	// client2->connect("192.168.1.112", 9016);
	// client->connect("192.168.1.107", 9016); // ODO 9016 
    // client.connect("192.168.0.4", 9012);
    // client.connect("192.168.0.4", 6060);

    char rbuf[4096];
    memset(rbuf, 0, sizeof(rbuf));
    int cmd  = -1; 
    bool m_close = false;

#ifdef WIN32
	HANDLE thread_record;
	HANDLE thread_send;
	HANDLE thread_realtime;
	DWORD thread_record_id;
	DWORD thread_send_id;
	DWORD thread_realtime_id;
#else
    pthread_t xtion_thread_record;
    pthread_t xtion_thread_send;
#endif

	// create realtime Client 
	OnlineSLAMClient * pRTClient = new OnlineSLAMClient;
	pRTClient->OnlineSLAMClientInit();
	//thread_realtime = CreateThread(0,0,
	//	(LPTHREAD_START_ROUTINE)&OnlineSLAMClient::OnlineSLAMClientRun(),
	//	(LPVOID)pRTClient, 0, &thread_realtime_id);
	cout<<"BEFORE RUN RTClient!"<<endl;
	cout<<fflush;
	Sleep(10);
	pRTClient->OnlineSLAMClientRun();
	cout<<"AFTER RUN RTClient!"<<endl;
	cout<<fflush;
	// syn time -> record data -> upload data 
    while(client->recvData(rbuf, sizeof(rbuf)))
    {
        memcpy(&cmd, rbuf, sizeof(4));
		cout<<"rece cmd: "<<cmd<<endl;
        switch(cmd)
        {
            case 0: 
                cout<<"cmd = 0, OK, let's asy our time!"<<endl;
                unsigned long sec, usec; 
				memcpy(&sec, rbuf +4, sizeof(unsigned long));
				memcpy(&usec, rbuf +4 +sizeof(unsigned long), sizeof(unsigned long));
				g_dwHighDateTime =  100000; // sec; 
				g_dwLowDateTime = 0; //usec;
				g_has_time_asy = true;
				cout<<"client rece asy time: "<<sec<<"."<<usec<<endl;
                break;
            case 1:
                cout<<"cmd = 1, OK, let's record data to local disk!"<<endl;
				thread_record = CreateThread(0,0, (LPTHREAD_START_ROUTINE)&CSocketClient::thread_start_record, \
					(LPVOID)client,0,&thread_record_id);
				if(thread_record == NULL)
				{
					cout<<"failed to create thread to record data locally!"<<endl;
				}else{
					cout<<"succeed to create thread to record data locally!"<<endl;
				}
				Sleep(100);
                break; 
            case 2:
                cout<<"cmd = 2, OK, let's stop send xtion data!"<<endl;
                client->stopRecordData();
                // client.stopSendXtion();
                break; 
            case 3:
                cout<<"cmd = 3, OK, let's clear all the data in the DIR!"<<endl;
                m_close = true;
                break; 
            case 4:
                cout<<"cmd = 4, OK, let's send all the data to server offline!"<<endl;
				thread_send = CreateThread(0,0, (LPTHREAD_START_ROUTINE)&CSocketClient::thread_start_send, \
							(LPVOID)client,0,&thread_send_id);
				if(thread_send == NULL)
				{
					cout<<"failed to create thread to send data to server!"<<endl;
				}else
				{
					cout<<"succeed to create thread to send data to server!"<<endl;
					Sleep(20);
					while(1)
					{
						if(client->sendThreadStatus())
						{
							Sleep(100);
						}
					}
				}
				
                Sleep(10);
                break;
			case 5:
				cout<<"cmd = 5, OK, let's send data blockedly!"<<endl;
				
				break;
            default:
                cout<<"Oh, no, Unknown cmd: "<<cmd<<" let's close"<<endl;
                m_close = true;
                break;
        }
        if(m_close ) break;
		Sleep(1); // sleep 1ms
        // usleep(100);
    }
	delete client;
	delete pRTClient;
}
/*
void test_socket_client()
{
    CSocketClient client; 
    client.connect("127.0.0.1", 6060);
    char rbuf[4096]; 
    char sbuf[4096];
    memset(sbuf, 0, sizeof(sbuf)); 
    memset(rbuf, 0, sizeof(rbuf));
    char* sendMsg = "Hello server, now I can only send you This!";
    memcpy(sbuf, sendMsg, strlen(sendMsg));
    int cmd = -1;
    bool bclose = false;
    double timestamp = 0;

    // receive data from server 
    while(client.recvData(rbuf, sizeof(rbuf)))
    {
        memcpy(&cmd, rbuf, 4);
        switch(cmd)
        {
            case 0: 
                memcpy(&timestamp, rbuf+4, sizeof(double));
                cout<<"cmd = 0: time asy! recv timestamp: "<< timestamp<<endl;
                break; 
            case 1:
                cout<<"cmd = 1: begin send data!"<<endl;
                client.sendData(sbuf, strlen(sbuf));
                break; 
            case 2:
                cout<<"cmd = 2: stop send data!"<<endl; 
                break; 
            case 3:
                cout<<"cmd = 3: send all data to server !"<<endl;
                break; 
            default:
                cout<<"unknown cmd = "<<cmd<<" now close connection!"<<endl;
                bclose = true;
                break;
        }
        if(bclose) break;
    }
}
*/