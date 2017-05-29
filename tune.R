#
# Try the staged CEM results.
# Not great.
#
d <- transform(
  read.csv('data/tune_6.csv'),
  crashed = crashed == 'true'
)
nrow(d)

# Require that we never crashed.
dOK <- subset(d, crashed == 0)
nrow(dOK)

# Timid: smallest error among our solutions
head(dOK[order(dOK$total_absolute_cte),])

# Aggressive: fastest laps
head(dOK[order(-dOK$distance),])
