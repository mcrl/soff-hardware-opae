#!/bin/sh

##
## This script plots the results generated by run_perf.sh
##

platform="SKX"
if [ -n "${1}" ]; then
    platform="${1}_"
fi

for mcl in 1 2 4
do
    for vc in 0 1 2
    do
        gnuplot -e "title='${platform} MCL=${mcl} VC=${vc}'" -e "datafile='stats/perf_mcl${mcl}_vc${vc}.dat'" -e "ofile='mcl${mcl}_vc${vc}'" scripts/plot_perf.gp
    done

    gnuplot -e "title='${platform} MCL=${mcl} VC Map'" -e "datafile='stats/perf_map_mcl${mcl}_vc0.dat'" -e "ofile='mcl${mcl}_vc0_map'" scripts/plot_perf.gp
done

# Crop whitespace
for fn in read_*.pdf write_*.pdf rw_*.pdf
do
    pdfcrop --margins 10 ${fn} crop_${fn} >/dev/null
    mv -f crop_${fn} ${fn}
done

# Bookmarks
cat >/tmp/pdfmark.$$ <<EOF
[/PageMode /UseOutlines /Page 1 /DOCVIEW pdfmark
[/Count 3 /Page 1 /Title (Read Bandwidth) /OUT pdfmark
[/Page 1 /Title (MCL=1) /OUT pdfmark
[/Page 5 /Title (MCL=2) /OUT pdfmark
[/Page 9 /Title (MCL=4) /OUT pdfmark
[/Count 3 /Page 13 /Title (Write Bandwidth) /OUT pdfmark
[/Page 13 /Title (MCL=1) /OUT pdfmark
[/Page 17 /Title (MCL=2) /OUT pdfmark
[/Page 21 /Title (MCL=4) /OUT pdfmark
[/Count 3 /Page 25 /Title (Read+Write Bandwidth) /OUT pdfmark
[/Page 25 /Title (MCL=1) /OUT pdfmark
[/Page 29 /Title (MCL=2) /OUT pdfmark
[/Page 33 /Title (MCL=4) /OUT pdfmark
EOF

# Merge into a single PDF
gs -dBATCH -dNOPAUSE -q -sDEVICE=pdfwrite -dPDFSETTINGS=/prepress -dCompatibilityLevel=1.4 -dUseCIEColor -sOutputFile=perf.pdf read_*.pdf write_*.pdf rw_*.pdf /tmp/pdfmark.$$
rm read_*.pdf write_*.pdf rw_*.pdf
rm /tmp/pdfmark.$$
