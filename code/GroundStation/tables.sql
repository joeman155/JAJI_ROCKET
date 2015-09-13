CREATE TABLE messages_t (id integer primary key autoincrement, 
                         instance_id varchar2(100),
                         message text, 
                         creation_date datetime);

CREATE TABLE measurement_group_t (
		id integer primary key autoincrement,
		group_name varchar2(5),
		source varchar2(10),
		creation_date datetime);

CREATE TABLE measurement_t (id integer primary key autoincrement,
                         instance_id varchar2(100),
			 group_id integer,
                         name   varchar2(20),
                         value float);


CREATE TABLE gps_t (id integer primary key autoincrement,
                    instance_id varchar2(100),
                    latitude float,
                    longitude float,
                    height    integer,
		    speed     float,
		    course    float,
                    satellites integer,
                    gps_date  varchar2(20),
                    gps_time  varchar2(20),
                    creation_date datetime);



CREATE TABLE imu_t (id integer primary key autoincrement,
                    instance_id varchar2(100),
                    roll   float,
                    pitch  float,
                    yaw    float,
                    gyrox     float,
                    gyroy     float,
                    gyroz     float,
                    accx      float,
                    accy      float,
                    accz      float,
                    accz      float,
                    timer     integer,
                    creation_date datetime);


CREATE TABLE gs_psu_voltage_t (id integer primary key autoincrement,
	                   instance_id varchar2(100),
                           psu_id integer,
			   voltage float,
			   creation_date datetime);


CREATE TABLE radio_stats_t (id integer primary key autoincrement,
                             instance_id varchar2(100),
			     place integer,
                             stats text,
                             creation_date datetime);


CREATE TABLE heartbeat_t (id integer primary key autoincrement,
	                  instance_id varchar2(100),
			  heartbeat integer,
			  creation_date datetime);
				                             

CREATE TABLE gps_prediction_t (id integer primary key autoincrement,
		    dtime integer,
                    latitude float,
                    longitude float,
                    height    integer,
                    creation_date datetime);

CREATE TABLE requests_t (id integer primary key autoincrement,
                    source varchar2(10),
                    request_code varchar2(150),
                    destination varchar2(10),
                    ip varchar2(15),
                    status_code integer,
                    notes varchar2(150),
                    creation_date datetime,
                    last_update_date datetime);

CREATE TABLE request_status_t (status_code char(1) primary key,
                               status_name varchar2(15),
                               status_desc varchar2(150));


CREATE TABLE request_types_t (request_code char(1) primary key,
                              request_name varchar2(15));

CREATE TABLE launch_system_status_t (id integer primary key autoincrement,
           attribute char(1),
           status   integer,
           notes    varchar2(150),
           creation_date datetime);


INSERT INTO request_status_t (status_code, status_name)
VALUES ('C', 'Created');

INSERT INTO request_status_t (status_code, status_name)
VALUES ('P', 'Processing');

INSERT INTO request_status_t (status_code, status_name)
VALUES ('F', 'Finished');



INSERT INTO request_types_t (request_code, request_name) VALUES ('P', 'Power');

INSERT INTO request_types_t (request_code, request_name) VALUES ('A', 'Arm');

INSERT INTO request_types_t (request_code, request_name) VALUES ('C', 'Continuity Test');

INSERT INTO request_types_t (request_code, request_name) VALUES ('T', 'Invalidate Continuity Test');

INSERT INTO request_types_t (request_code, request_name) VALUES ('L', 'Launch');

INSERT INTO request_types_t (request_code, request_name) VALUES ('M', 'Resetting previous Launch status');

INSERT INTO request_types_t (request_code, request_name) VALUES ('N', 'NoPhotos');

INSERT INTO request_types_t (request_code, request_name) VALUES ('K', 'Cutdown');

INSERT INTO request_types_t (request_code, request_name) VALUES ('X', 'Xmodem Download');


