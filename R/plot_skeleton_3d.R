# plot_skeleton_3D()
#
# NSL Analysis (https://github.com/alinen/rit-nsl-analysis)
# Aline Normoyle, anormoyle@brynmawr.edu
#
# 3D Animation of positions data

plot_skeleton_3d = function(D, participant, f) {

# We'll be using plotly for this, so let's require it
if (!requireNamespace("plotly", quietly = TRUE)) {
  stop("Package \"plotly\" needed for this function to work. Please install it.", call. = FALSE)
} else {
  library("plotly")
}

ids <- c(rwrist, relbow, rshoulder, neck, head, neck, lshoulder, lelbow, lwrist)

x <- D[f,(ids-1)*3+1]
y <- D[f,(ids-1)*3+2]
z <- D[f,(ids-1)*3+3]

fig <- plot_ly(x = as.vector(unlist(x)), 
               y = as.vector(unlist(y)), 
               z = as.vector(unlist(z)), 
               type = 'scatter3d', mode = 'lines+markers', showlegend = F,  
               line = list(width = 6, color = 'rgb(44, 160, 44)'),
               marker = list(size = 3.5, color = 'rgb(0, 0, 255)'))

xs <- D[,(ids-1)*3+1]
ys <- D[,(ids-1)*3+2]
zs <- D[,(ids-1)*3+3]

fig <- fig %>%
  layout(
    title= sprintf("%s Frame: %d (%.2fs)", participant, f, f/30),
    scene= list(
      camera= list(
        eye = list(x=0, y = 0.5, z = -2.5),
        up= list(x= 0, y= 1, z= 0)
      ),
      xaxis= list(
        range = c(min(xs),max(xs))
      ),
      yaxis= list(
        range = c(min(ys), max(ys))
      ),
      zaxis= list(
        range = c(min(zs), max(zs))
      )
    )
  )
fig
return(fig)
}