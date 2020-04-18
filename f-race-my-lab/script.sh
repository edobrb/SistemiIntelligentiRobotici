#!/bin/bash

for i in {1..1}
do
   argos3 -c test-ptoa.argos | tail -n2  | head -n1 | grep -o -P '(?<=ABC).*(?=CBA)'
done

