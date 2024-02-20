document.addEventListener('DOMContentLoaded', function () {
    const sendRequestButton = document.getElementById('sendRequestButton');
    const responseText = document.getElementById('responseText');

    sendRequestButton.addEventListener('click', function () {
        // Perform a GET request using Axios
        axios.get('http://192.168.137.1:8080') // Replace 'YOUR_API_ENDPOINT' with the actual API endpoint
            .then(response => {
                // Display the received text in the span
                responseText.textContent = response.data;
            })
            .catch(error => {
                console.error('Error:', error);
                responseText.textContent = 'Error occurred while fetching data.';
            });
    });
});
