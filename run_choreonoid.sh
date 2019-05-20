#!/bin/bash

#pkill -9 choreonoid

echo $@ >&2

choreonoid $@
