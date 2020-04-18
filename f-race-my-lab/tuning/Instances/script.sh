#!/bin/bash

for i in "$@"
do
case $i in
    -e=*|--FORWARD_SCHEMA_M=*)
    FORWARD_SCHEMA_M="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--OBSTACLE_SCHEMA_M=*)
    OBSTACLE_SCHEMA_M="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--OBSTACLE_CIRCUM_SCHEMA_M=*)
    OBSTACLE_CIRCUM_SCHEMA_M="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--LIGHT_CIRCUM_SCHEMA_M=*)
    LIGHT_CIRCUM_SCHEMA_M="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--LIGHT_CIRCUM_SCHEMA_T=*)
    LIGHT_CIRCUM_SCHEMA_T="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--OBSTACLE_SCHEMA_T=*)
    OBSTACLE_SCHEMA_T="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--OBSTACLE_CIRCUM_SCHEMA_T=*)
    OBSTACLE_CIRCUM_SCHEMA_T="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--PROXIMITY_MAX_ALPHA=*)
    PROXIMITY_MAX_ALPHA="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--PROXIMITY_AVG_ALPHA=*)
    PROXIMITY_AVG_ALPHA="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--LIGHT_ALPHA=*)
    LIGHT_ALPHA="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--ALMOST_TANGENT=*)
    ALMOST_TANGENT="${i#*=}"
    shift # past argument=value
    ;;
	-e=*|--run=*)
    RUN="${i#*=}"
    shift # past argument=value
    ;;
esac
done

ARGOS_FILE="tmp.argos"
sed -e "s/\${FORWARD_SCHEMA_M}/$FORWARD_SCHEMA_M/" -e "s/\${OBSTACLE_SCHEMA_M}/$OBSTACLE_SCHEMA_M/" -e "s/\${OBSTACLE_CIRCUM_SCHEMA_M}/$OBSTACLE_CIRCUM_SCHEMA_M/" -e "s/\${LIGHT_CIRCUM_SCHEMA_M}/$LIGHT_CIRCUM_SCHEMA_M/" -e "s/\${OBSTACLE_SCHEMA_T}/$OBSTACLE_SCHEMA_T/" -e "s/\${OBSTACLE_CIRCUM_SCHEMA_T}/$OBSTACLE_CIRCUM_SCHEMA_T/" -e "s/\${LIGHT_CIRCUM_SCHEMA_T}/$LIGHT_CIRCUM_SCHEMA_T/" -e "s/\${PROXIMITY_MAX_ALPHA}/$PROXIMITY_MAX_ALPHA/" -e "s/\${PROXIMITY_AVG_ALPHA}/$PROXIMITY_AVG_ALPHA/" -e "s/\${LIGHT_ALPHA}/$LIGHT_ALPHA/" -e "s/\${ALMOST_TANGENT}/$ALMOST_TANGENT/" $RUN  > $ARGOS_FILE


argos3 -n -c $ARGOS_FILE

rm $ARGOS_FILE
