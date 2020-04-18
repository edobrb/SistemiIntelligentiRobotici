#!/bin/bash

for i in {1..100}
do
   argos3 -c test-ptoa.argos | tail -n2  | head -n1 | grep -o -P '(?<=ABC).*(?=CBA)'
done

