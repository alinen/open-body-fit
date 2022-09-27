magnitudes <- function(D) {
  
  end <- ncol(D)
  x <- D[,seq(1,end,3)]
  y <- D[,seq(2,end,3)]
  z <- D[,seq(3,end,3)]
  
  M = matrix(0, nrow(x), ncol(x))
  for (i in 1:nrow(M)) {
    for (j in 1:ncol(M)) {
      M[i,j] = sqrt(x[i,j]*x[i,j] + y[i,j]*y[i,j] + z[i,j]*z[i,j])
    }
  }
  colnames(M) <- c("root", "neck", "head", "torso", 
                      "lshoulder", "lelbow", "lwrist", "lhand", 
                      "rshoulder", "relbow", "rwrist", "rhand")
  rownames(M) <- 1:nrow(x)
  M <- as.data.frame(M)
  return(M)
  
}