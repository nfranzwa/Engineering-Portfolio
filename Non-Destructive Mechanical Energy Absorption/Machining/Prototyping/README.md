<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Side by Side Videos</title>
    <style>
        .video-container {
            display: flex;
            justify-content: center;
            gap: 20px;
        }
        .video-container video {
            width: 45%;
            height: auto;
        }
    </style>
</head>
<body>
    <div class="video-container">
        <video controls>
            <source src="https://github.com/user-attachments/assets/b42999ff-ae61-42d9-b68b-e1c38a5ad360" type="video/mp4">
            Your browser does not support the video tag.
        </video>
        <video controls>
            <source src="https://github.com/user-attachments/assets/1c2a48b7-b4d0-4edd-bec2-8a13a5aa5105" type="video/mp4">
            Your browser does not support the video tag.
        </video>
    </div>
</body>
</html>
