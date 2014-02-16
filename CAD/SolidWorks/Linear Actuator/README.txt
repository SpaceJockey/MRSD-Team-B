Heirarchy as such:
FixedEnd.sldprt -> basic part mass
|-FixedEndJoin.sldasm -> Geared end joined with assembly screw hole feature added, use this for Larger Space Jockey Assembly without added complexity
  |-FixedEndJoin.sldprt -> partified version of above
    |-FixedEndSplit.sldasm -> Adds manufacturing features, Split + ridge & groove (turns out, Solidworks has a feature for that)
      |-Top.sldpart -> rendered top part for printing
      |-Bottom.sldprt -> rendered bottom part for printing