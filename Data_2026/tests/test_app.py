def test_index_page(client):
    response = client.get("/")
    assert response.status_code == 200
    assert b"JSON" in response.data
    assert b"Image" in response.data
    assert b"Error" in response.data


def test_error_page(client):
    response = client.get("/error")
    assert response.status_code == 200
    assert b"error" in response.data.lower()


def test_image_page(client):
    response = client.get("/image")
    assert response.status_code == 200
    assert b"<img" in response.data


