#!/bin/sh

rm -f hx8k_65xx_top.asc
rm -rf runs
mkdir -p runs

number=1 maxfreq=0 ; while test $number -le 10 ; do
    make hx8k_65xx_top.asc 2> runs/log$number
    freq=$(make hx8k_65xx_top.rpt | grep estimate | cut -d ' ' -f 6 | cut -d '(' -f 2)
    mv hx8k_65xx_top.asc runs/asc$number
    mv hx8k_65xx_top.rpt runs/rpt$number
    if expr $freq '>' $maxfreq ; then
        maxfreq=$freq
        maxnum=$number
    fi
    echo "$number: Current frequency is $freq, current max frequency is $maxfreq"
    number=$((number+1))
done

echo "Found max frequency $maxfreq in run $maxnum"
cp runs/asc$maxnum hx8k_65xx_top.asc
cp runs/rpt$maxnum hx8k_65xx_top.rpt
