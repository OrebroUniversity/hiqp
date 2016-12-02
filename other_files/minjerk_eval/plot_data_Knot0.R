library(ggplot2)

source("multiplot.R")

rawdata <- read.csv("f5t5Knot0_results.csv", header=FALSE)

p1 <- ggplot(data.frame(x=log(rawdata$V3), y=rawdata$V4), aes(x=x, y=y)) + geom_line() +
  xlab("log of feed-forward gain K []") + ylab("final error [%]") + labs(title = "Rate of task fulfillment")

p2 <- ggplot(data.frame(x=log(rawdata$V3), y=rawdata$V5), aes(x=x, y=y)) + geom_line() +
  xlab("log of feed-forward gain K []") + ylab("Sigma over C") + labs(title = "Rate of jerk consistency")

multiplot(p1, p2, cols=2)

#grid.arrange(p1, p2, ncol = 2, main = "Main title")
