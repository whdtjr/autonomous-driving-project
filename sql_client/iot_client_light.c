#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <mysql/mysql.h>

#define BUF_SIZE 100
#define NAME_SIZE 20
#define ARR_CNT 5

void* send_msg(void* arg);
void* recv_msg(void* arg);
void error_handling(char* msg);

char name[NAME_SIZE] = "[Default]";
char msg[BUF_SIZE];

int main(int argc, char* argv[])
{
	int sock;
	struct sockaddr_in serv_addr;
	pthread_t snd_thread, rcv_thread, mysql_thread;
	void* thread_return;

	if (argc != 4) {
		printf("Usage : %s <IP> <port> <name>\n", argv[0]);
		exit(1);
	}

	sprintf(name, "%s", argv[3]);

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock == -1)
		error_handling("socket() error");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(atoi(argv[2]));

	if (connect(sock, (struct sockaddr*) & serv_addr, sizeof(serv_addr)) == -1)
		error_handling("connect() error");

	sprintf(msg, "[%s:PASSWD]", name);
	write(sock, msg, strlen(msg));
	pthread_create(&rcv_thread, NULL, recv_msg, (void*)&sock);
	pthread_create(&snd_thread, NULL, send_msg, (void*)&sock);


	pthread_join(snd_thread, &thread_return);
	pthread_join(rcv_thread, &thread_return);

	close(sock);
	return 0;
}


void* send_msg(void* arg)
{
	int* sock = (int*)arg;
	int str_len;
	int ret;
	fd_set initset, newset;
	struct timeval tv;
	char name_msg[NAME_SIZE + BUF_SIZE + 2];

	FD_ZERO(&initset);
	FD_SET(STDIN_FILENO, &initset);

	fputs("Input a message! [ID]msg (Default ID:ALLMSG)\n", stdout);
	while (1) {
		memset(msg, 0, sizeof(msg));
		name_msg[0] = '\0';
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		newset = initset;
		ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);
		if (FD_ISSET(STDIN_FILENO, &newset))
		{
			fgets(msg, BUF_SIZE, stdin);
			if (!strncmp(msg, "quit\n", 5)) {
				*sock = -1;
				return NULL;
			}
			else if (msg[0] != '[')
			{
				strcat(name_msg, "[ALLMSG]");
				strcat(name_msg, msg);
			}
			else
				strcpy(name_msg, msg);
			if (write(*sock, name_msg, strlen(name_msg)) <= 0)
			{
				*sock = -1;
				return NULL;
			}
		}
		if (ret == 0)
		{
			if (*sock == -1)
				return NULL;
		}
	}
}

void* recv_msg(void* arg)
{
	MYSQL* conn;
	MYSQL_ROW sqlrow;
	int res;
	char sql_cmd[200] = { 0 };
	char socket_cmd[200] = { 0 };

	// db 접속 정보
	char* host = "localhost";
	char* user = "v2x";
	char* pass = "pwv2x";
	char* dbname = "v2x_platform";

	int* sock = (int*)arg;
	int i;
	char* pToken;
	char* pArray[ARR_CNT] = { 0 };

	char name_msg[NAME_SIZE + BUF_SIZE + 1];
	int str_len;
	int time_led;
	char state_led[10];

	conn = mysql_init(NULL);

	puts("MYSQL startup");
	if (!(mysql_real_connect(conn, host, user, pass, dbname, 0, NULL, 0)))
	{
		fprintf(stderr, "ERROR : %s[%d]\n", mysql_error(conn), mysql_errno(conn));
		exit(1);
	}
	else
		printf("Connection Successful!\n\n");

	while (1) {
		memset(name_msg, 0x0, sizeof(name_msg));
		str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE);
		if (str_len <= 0)
		{
			*sock = -1;
			return NULL;
		}
		fputs(name_msg, stdout);
		name_msg[strcspn(name_msg,"\n")] = '\0';

		pToken = strtok(name_msg, "[:@]");
		i = 0;
		while (pToken != NULL)
		{
			pArray[i] = pToken;
			if ( ++i >= ARR_CNT)
				break;
			pToken = strtok(NULL, "[:@]");

		}

//--------------time_light UPDATE------------------		
		// V2I_STM -> [V2I_DB]TIME@20@RED@Z1
		// 수신 msg : [V2I_STM]TIME@20@RED@Z1
		// V2I_DB -> [V2I_UI]TIME@20@RED@Z1
		// V2I_DB -> [ROS2]RED
		if(!strcmp(pArray[1],"TIME") && (i == 5)){
			time_led = atoi(pArray[2]);

			// V2I_STM한테 받은 신호등 데이터를 DB Table에 업데이트
			sprintf(sql_cmd, "update vehicle_zone set time_led='%d', state_led='%s' where zone_id='%s'", time_led, pArray[3], pArray[4]);
			res = mysql_query(conn, sql_cmd);
			if (!res)
				printf("MEMBER : updated %lu rows\n", (unsigned long)mysql_affected_rows(conn));

			// QT한테 전송: [V2I_UI]TIME@20@RED@Z1 
			sprintf(socket_cmd, "[%s]%s@%s@%s@%s\n","V2I_UI",pArray[1],pArray[2],pArray[3],pArray[4]);
			write(*sock, socket_cmd, strlen(socket_cmd));

			// ROS2한테 전송: [ROS2]RED
			sprintf(socket_cmd, "[%s]%s\n","ROS2",pArray[3]);
			write(*sock, socket_cmd, strlen(socket_cmd));
			
		}	
//---------------지자기 인식되면 table에 차량 추가--------------		
		// ROS2 -> [V2I_DB]IN
		// 수신 msg : [ROS2]IN
		if(!strcmp(pArray[1], "IN") && (i == 2)){
  			sprintf(sql_cmd, "insert into vehicle_zone(zone_id, vehicle_id) values(\"%s\",\"%s\")","Z1",pArray[0]);
			// printf("[IN]sql_cmd : %s\n", sql_cmd);
			res = mysql_query(conn, sql_cmd);
			if (!res)
				printf("MEMBER : inserted %lu rows\n", (unsigned long)mysql_affected_rows(conn));
		}	
//---------------지자기 인식 안되면 table에서 차량 삭제------------		
		// ROS2 -> [V2I_DB]OUT
		// 수신 msg : [ROS2]OUT
		if(!strcmp(pArray[1], "OUT") && (i == 2)){
  			sprintf(sql_cmd, "delete from vehicle_zone where vehicle_id='%s'", pArray[0]);
			// printf("[OUT]sql_cmd : %s\n", sql_cmd);
			res = mysql_query(conn, sql_cmd);
			if (!res)
				printf("MEMBER : deleted %lu rows\n", (unsigned long)mysql_affected_rows(conn));
		}	
//--------------------------------------------------------------
		else 
			continue;
	}
	mysql_close(conn);
}

void error_handling(char* msg)
{
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(1);
}
