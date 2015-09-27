DROP TABLE imu_t;
DROP sequence imu_id_seq;

CREATE sequence imu_id_seq;

CREATE TABLE imu_t (id integer primary key default nextval('imu_id_seq'),
                    instance_id varchar(100),
                    roll   float,
                    pitch  float,
                    yaw    float,
                    gyrox     float,
                    gyroy     float,
                    gyroz     float,
                    accx      float,
                    accy      float,
                    accz      float,
                    timer     integer,
                    creation_date date);

ALTER TABLE imu_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on imu_t to "www-data";

DROP TABLE measurement_group_t;
DROP SEQUENCE measurement_group_id_seq;

CREATE sequence measurement_group_id_seq;

CREATE TABLE measurement_group_t (
    id integer primary key default nextval('measurement_group_id_seq'),
    group_name varchar(5),
    source varchar(10),
    creation_date date);

ALTER TABLE measurement_group_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on measurement_group_t to "www-data";



DROP TABLE measurement_t;
DROP SEQUENCE measurement_id_seq;

CREATE sequence measurement_id_seq;

CREATE TABLE measurement_t (id integer primary key default nextval('measurement_id_seq'),
    instance_id varchar(100),
    group_id integer,
    name   varchar(20),
    value float);

grant select on measurement_t to "www-data";



DROP TABLE messages_t;
DROP SEQUENCE message_id_seq;

CREATE sequence message_id_seq;

CREATE TABLE messages_t (id integer primary key default nextval('message_id_seq'),
                         instance_id varchar(100),
                         message varchar(256),
                         creation_date date);

ALTER TABLE messages_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on messages_t to "www-data";


DROP TABLE gps_t;
DROP SEQUENCE gps_id_seq;

CREATE sequence gps_id_seq;


CREATE TABLE gps_t (id integer primary key default nextval('gps_id_seq'),
                    instance_id varchar(100),
                    latitude float,
                    longitude float,
                    height    integer,
		    speed     float,
		    course    float,
                    satellites integer,
                    gps_date  varchar(20),
                    gps_time  varchar(20),
                    creation_date date);

ALTER TABLE gps_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on gps_t to "www-data";

DROP TABLE gs_psu_voltage_t;
DROP SEQUENCE gs_psu_voltage_id_seq;

CREATE sequence gs_psu_voltage_id_seq;

CREATE TABLE gs_psu_voltage_t (id integer primary key default nextval('gs_psu_voltage_id_seq'),
                           instance_id varchar(100),
                           psu_id integer,
                           voltage float,
                           creation_date date);

ALTER TABLE gs_psu_voltage_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on gs_psu_voltage_t to "www-data";

DROP TABLE radio_stats_t;
DROP SEQUENCE radio_stats_id_seq;

CREATE sequence radio_stats_id_seq;

CREATE TABLE radio_stats_t (id integer primary key default nextval('radio_stats_id_seq'),
                             instance_id varchar(100),
                             place integer,
                             stats text,
                             creation_date date);

ALTER TABLE radio_stats_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on radio_stats_t to "www-data";


DROP TABLE heartbeat_t;
DROP SEQUENCE heartbeat_id_seq;

CREATE sequence heartbeat_id_seq;

CREATE TABLE heartbeat_t (id integer primary key default nextval('heartbeat_id_seq'),
                          instance_id varchar(100),
                          heartbeat integer,
                          creation_date date);

ALTER TABLE heartbeat_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on heartbeat_t to "www-data";

DROP TABLE gps_prediction_t;
DROP SEQUENCE  gps_prediction_id_seq;

CREATE sequence gps_prediction_id_seq;


CREATE TABLE gps_prediction_t (id integer primary key default nextval('gps_prediction_id_seq'),
                    dtime integer,
                    latitude float,
                    longitude float,
                    height    integer,
                    creation_date date);

ALTER TABLE gps_prediction_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on gps_prediction_t to "www-data";

DROP TABLE requests_t;
DROP SEQUENCE requests_id_seq;

CREATE sequence requests_id_seq;

CREATE TABLE requests_t (id integer primary key default nextval('requests_id_seq'),
                    source varchar(10),
                    request_code varchar(150),
                    destination varchar(10),
                    ip varchar(15),
                    status_code integer,
                    notes varchar(150),
                    creation_date date,
                    last_update_date date);

ALTER TABLE requests_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on requests_t to "www-data";

DROP TABLE request_status_t;

CREATE TABLE request_status_t (status_code char(1) primary key,
                               status_name varchar(15),
                               status_desc varchar(150));

grant select on request_status_t to "www-data";


DROP TABLE request_types_t;

CREATE TABLE request_types_t (request_code char(1) primary key,
                              request_name varchar(50));

grant select on request_types_t to "www-data";


DROP TABLE launch_system_status_t;
DROP SEQUENCE launch_system_status_id_seq;

CREATE sequence launch_system_status_id_seq;

CREATE TABLE launch_system_status_t (id integer primary key default nextval('launch_system_status_id_seq'),
           attribute char(1),
           status   integer,
           notes    varchar(150),
           creation_date date);

ALTER TABLE launch_system_status_t ALTER COLUMN creation_date SET DEFAULT CURRENT_DATE;

grant select on launch_system_status_t to "www-data";


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
