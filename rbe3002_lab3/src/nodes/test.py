#!/usr/bin/env python


padded_grid = []
        ## Calculate the kernel based on padding param
        kernel = []
        size = 1 + 2*int(padding)
        for y in range(size):
            row = []
            for x in range(size):
                row.append(1)
            kernel.append(row)

            