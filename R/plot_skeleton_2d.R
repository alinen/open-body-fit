# plot_skeleton_2D()
#
# NSL Analysis (https://github.com/alinen/rit-nsl-analysis)
# Aline Normoyle, anormoyle@brynmawr.edu
#
# Plots a single frame of 3D data onto the corresponding video image

plot_skeleton_2d <- function(raw_img, C, frame, D, participant, fps = 30.0) {
  
  ids <- c(rwrist, relbow, rshoulder, neck, head, neck, lshoulder, lelbow, lwrist)
  
  x <- D[frame,(ids-1)*3+1]
  y <- D[frame,(ids-1)*3+2]
  z <- D[frame,(ids-1)*3+3]
  
  sx <- matrix(0, nrow=nrow(x), ncol=ncol(x))
  sy <- matrix(0, nrow=nrow(x), ncol=ncol(x))
  
  pos3d <- matrix(1, nrow=4, ncol=ncol(x));
  pos3d[1,] <- as.vector(unlist(x));
  pos3d[2,] <- as.vector(unlist(y));
  pos3d[3,] <- as.vector(unlist(z));
  
  pos3d <- t(C) %*% pos3d;
  sx <- pos3d[1,] / pos3d[3,];
  sy <- pos3d[2,] / pos3d[3,];
  

  title = sprintf("%s: %d (%.1fs)", participant, frame, frame/fps)
  
  fig <- plot_ly(type="image", z=raw_img*255)
  fig <- add_paths(fig, x = as.vector(unlist(sx[1:3])), 
                   y = as.vector(unlist(sy[1:3])), 
                   line = list(width = 4, color = 'rgb(44, 160, 44)'),
                   inherit=F)
  fig <- add_paths(fig, x = as.vector(unlist(sx[3:7])), 
                   y = as.vector(unlist(sy[3:7])), 
                   line = list(width = 4, color = 'rgb(44, 160, 255)'),
                   inherit=F)
  fig <- add_paths(fig, x = as.vector(unlist(sx[7:9])), 
                   y = as.vector(unlist(sy[7:9])), 
                   line = list(width = 4, color = 'rgb(255, 160, 44)'),
                   inherit=F)
  fig <- add_markers(fig, x = as.vector(unlist(sx)), 
                     y = as.vector(unlist(sy)), 
                     marker = list(size = 6, color = 'rgb(44, 44, 44)'),
                     inherit=F)
  fig <- add_text(fig, x = 360, y = 20, text=title, inherit=F,
                  textfont = list(color = '#FFFFFF', size = 24))
  fig <- hide_legend(fig)
  fig
}