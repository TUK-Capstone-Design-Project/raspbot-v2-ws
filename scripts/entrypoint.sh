#!/bin/bash

chmod 666 /dev/i2c-*
chmod 666 /dev/dma_heap/*

exec "$@"
